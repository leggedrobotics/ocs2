//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <iostream>
#include <chrono>
#include <Eigen/QR>

// scalar_t is double
// matrix_t is Eigen::MatrixXd
// matrix_array_t is std::vector<Eigen::MatrixXd>

namespace ocs2
{

  MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings,
                                                 const SystemDynamicsBase *systemDynamicsPtr,
                                                 const CostFunctionBase *costFunctionPtr,
                                                 const ConstraintBase *constraintPtr)
      : Solver_BASE(),
        systemDynamicsPtr_(systemDynamicsPtr->clone()),
        costFunctionPtr_(costFunctionPtr->clone()),
        constraintPtr_(constraintPtr->clone()),
        settings_(std::move(settings))
  {
    std::cout << "creating multiple shooting solver\n";
  }

  void MultipleShootingSolver::reset()
  {
    // Solver_BASE::reset();
    // there is no Solve_BASE::reset() function. One can see GaussNewtonDDP.h. The reset function there only clears some variables of the solver itself.
    // additional reset
    std::cout << "resetting\n";
  }

  void MultipleShootingSolver::runImpl(scalar_t initTime,
                                       const vector_t &initState,
                                       scalar_t finalTime,
                                       const scalar_array_t &partitioningTimes)
  {
    // ignore partitioningTimes

    // Initialize cost
    costFunctionPtr_->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

    // Solve the problem.
    setupDimension();
    scalar_t delta_t_ = (finalTime - initTime) / settings_.N;
    matrix_t x(settings_.n_state, settings_.N + 1);
    matrix_t u(settings_.n_input, settings_.N);
    if (!settings_.initPrimalSol)
    {
      x = matrix_t::Random(settings_.n_state, settings_.N + 1);
      u = matrix_t::Random(settings_.n_input, settings_.N);
      std::cout << "using random init\n";
    }
    else
    {
      for (int i = 0; i < settings_.N; i++)
      {
        x.col(i) = primalSolution_.stateTrajectory_[i];
        u.col(i) = primalSolution_.inputTrajectory_[i];
      }
      std::cout << "using past primal sol init\n";
    }
    settings_.initPrimalSol = true;

    scalar_t sqpTimeAll = 0.0;
    for (int i = 0; i < settings_.sqpIteration; i++)
    {
      std::cout << "\n---------------sqp iteration " << i << "----------\n";
      auto startSqpTime = std::chrono::steady_clock::now();
      setupCostDynamicsEqualityConstraint(*systemDynamicsPtr_, *costFunctionPtr_, *constraintPtr_, delta_t_, initTime, x, u, initState);
      solveOCP();
      vector_t delta_x0 = initState - x.col(0);
      matrix_t delta_x, delta_u;
      std::tie(delta_x, delta_u) = getOCPSolution(delta_x0);
      freeHPIPMMem();
      x += delta_x; // step size will be used in future
      u += delta_u;
      for (int j = 0; j < settings_.N; j++)
      {
        std::cout << "x dot u is: " << x.col(j).dot(u.col(j)) << std::endl;
      }
      auto endSqpTime = std::chrono::steady_clock::now();
      auto sqpIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSqpTime - startSqpTime);
      scalar_t sqpTime = std::chrono::duration<scalar_t, std::milli>(sqpIntervalTime).count();
      sqpTimeAll += sqpTime;
      std::cout << "SQP time one iter: " << sqpTime << "[ms]." << std::endl;
    }

    std::cout << "SQP time total: " << sqpTimeAll << "[ms]." << std::endl;

    // Fill PrimalSolution. time, state , input
    scalar_array_t timeTrajectory;
    vector_array_t stateTrajectory;
    vector_array_t inputTrajectory;
    timeTrajectory.resize(settings_.N);
    stateTrajectory.resize(settings_.N);
    inputTrajectory.resize(settings_.N);
    for (int i = 0; i < settings_.N; i++)
    {
      timeTrajectory[i] = initTime + delta_t_ * i;
      stateTrajectory[i] = x.col(i);
      inputTrajectory[i] = u.col(i);
    }
    primalSolution_.timeTrajectory_ = timeTrajectory;
    primalSolution_.stateTrajectory_ = stateTrajectory;
    primalSolution_.inputTrajectory_ = inputTrajectory;
    primalSolution_.modeSchedule_ = this->getModeSchedule();
    primalSolution_.controllerPtr_.reset(new FeedforwardController(primalSolution_.timeTrajectory_, primalSolution_.inputTrajectory_));
  }

  void MultipleShootingSolver::solveOCP()
  {
    scalar_t *AA[settings_.N];
    scalar_t *BB[settings_.N];
    scalar_t *bb[settings_.N];
    scalar_t *QQ[settings_.N + 1];
    scalar_t *RR[settings_.N];
    scalar_t *SS[settings_.N];
    scalar_t *qq[settings_.N + 1];
    scalar_t *rr[settings_.N];

    scalar_t *CC[settings_.N];
    scalar_t *DD[settings_.N];
    scalar_t *llg[settings_.N];
    scalar_t *uug[settings_.N];

    for (int i = 0; i < settings_.N; i++)
    {
      AA[i] = A_data[i].data();
      BB[i] = B_data[i].data();
      bb[i] = b_data[i].data();
      QQ[i] = Q_data[i].data();
      RR[i] = R_data[i].data();
      SS[i] = S_data[i].data();
      qq[i] = q_data[i].data();
      rr[i] = r_data[i].data();
      if (settings_.constrained && !settings_.qr_decomp)
      {
        CC[i] = C_C_data[i].data();
        DD[i] = D_D_data[i].data();
        llg[i] = lg_data[i].data();
        uug[i] = ug_data[i].data();
      }
    }
    QQ[settings_.N] = Q_data[settings_.N].data();
    qq[settings_.N] = q_data[settings_.N].data();

    int dim_size = d_ocp_qp_dim_memsize(settings_.N);
    dim_mem = malloc(dim_size);
    d_ocp_qp_dim_create(settings_.N, &dim, dim_mem);
    int *nu = nu_.data();
    int *nx = nx_.data();
    int *nbu = nbu_.data();
    int *nbx = nbx_.data();
    int *ng = ng_.data();
    int *nsbx = nsbx_.data();
    int *nsbu = nsbu_.data();
    int *nsg = nsg_.data();
    d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

    int qp_size = d_ocp_qp_memsize(&dim);
    qp_mem = malloc(qp_size);
    d_ocp_qp_create(&dim, &qp, qp_mem);
    scalar_t **hA = AA;
    scalar_t **hB = BB;
    scalar_t **hb = bb;
    scalar_t **hQ = QQ;
    scalar_t **hR = RR;
    scalar_t **hS = SS;
    scalar_t **hq = qq;
    scalar_t **hr = rr;
    int **hidxbx = NULL;
    scalar_t **hlbx = NULL;
    scalar_t **hubx = NULL;
    int **hidxbu = NULL;
    scalar_t **hlbu = NULL;
    scalar_t **hubu = NULL;
    scalar_t **hC = CC;
    scalar_t **hD = DD;
    scalar_t **hlg = llg;
    scalar_t **hug = uug;
    scalar_t **hZl = NULL;
    scalar_t **hZu = NULL;
    scalar_t **hzl = NULL;
    scalar_t **hzu = NULL;
    int **hidxs = NULL;
    scalar_t **hlls = NULL;
    scalar_t **hlus = NULL;
    scalar_t **hu_guess = NULL;
    scalar_t **hx_guess = NULL;
    scalar_t **hsl_guess = NULL;
    scalar_t **hsu_guess = NULL;
    d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

    int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    qp_sol_mem = malloc(qp_sol_size);
    d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    ipm_arg_mem = malloc(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
    d_ocp_qp_ipm_arg_set_default(mode, &arg);
    // the hpipm parameters below are subject to changes, may be integratred into settings_
    int iter_max = 30;
    scalar_t alpha_min = 1e-8;
    scalar_t mu0 = 1e4;
    scalar_t tol_stat = 1e-5;
    scalar_t tol_eq = 1e-5;
    scalar_t tol_ineq = 1e-5;
    scalar_t tol_comp = 1e-5;
    scalar_t reg_prim = 1e-12;
    int warm_start = 0;
    int pred_corr = 1;
    int ric_alg = 0;
    d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
    d_ocp_qp_ipm_arg_set_alpha_min(&alpha_min, &arg);
    d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
    d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
    d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
    d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
    d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
    d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
    d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
    d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
    d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

    int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
    ipm_mem = malloc(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    hpipm_timer timer;
    hpipm_tic(&timer);
    d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
    d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
    scalar_t time_ipm = hpipm_toc(&timer);
    printf("\nSolution time: %e [s]\n", time_ipm);

    if (settings_.printSolverStatus)
    {
      printf("\nHPIPM returned with flag %i.\n", hpipm_status);
      if (hpipm_status == 0)
      {
        printf("\n -> QP solved!\n");
      }
      else if (hpipm_status == 1)
      {
        printf("\n -> Solver failed! Maximum number of iterations reached\n");
      }
      else if (hpipm_status == 2)
      {
        printf("\n -> Solver failed! Minimum step length reached\n");
      }
      else if (hpipm_status == 3)
      {
        printf("\n -> Solver failed! NaN in computations\n");
      }
      else
      {
        printf("\n -> Solver failed! Unknown return flag\n");
      }
    }

    if (settings_.printSolverStatistics)
    {
      int iter;
      d_ocp_qp_ipm_get_iter(&workspace, &iter);
      scalar_t res_stat;
      d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
      scalar_t res_eq;
      d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
      scalar_t res_ineq;
      d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
      scalar_t res_comp;
      d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
      scalar_t *stat;
      d_ocp_qp_ipm_get_stat(&workspace, &stat);
      int stat_m;
      d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);
      printf("\nipm return = %d\n", hpipm_status);
      printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);
      printf("\nipm iter = %d\n", iter);
      printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
      d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);
      printf("\nocp ipm time = %e [s]\n\n", time_ipm);
    }
  }

  std::tuple<matrix_t, matrix_t> MultipleShootingSolver::getOCPSolution(const vector_t &delta_x0)
  {
    // retrieve deltaX, deltaUTilde
    auto startFillTime = std::chrono::steady_clock::now();

    matrix_t deltaX(settings_.n_state, settings_.N + 1);
    deltaX.col(0) = delta_x0; // because the first variable delta x0 is not optimized. Rather it's a constant.
    matrix_t deltaU(settings_.n_input, settings_.N);
    // only useful for QR decomposition case
    matrix_t deltaUTilde(settings_.n_input - settings_.n_constraint, settings_.N); // now this is \tilde{\delta u}, only useful when constrained
    scalar_t uTemp[settings_.n_input];                                             // take the largest possible
    scalar_t xTemp[settings_.n_state];

    for (int i = 0; i < settings_.N; i++)
    {
      d_ocp_qp_sol_get_x(i + 1, &qp_sol, xTemp);
      for (int j = 0; j < settings_.n_state; j++)
      {
        deltaX(j, i + 1) = xTemp[j];
      }
      d_ocp_qp_sol_get_u(i, &qp_sol, uTemp);
      if (settings_.constrained && settings_.qr_decomp)
      {
        for (int j = 0; j < settings_.n_input - settings_.n_constraint; j++) // <-- pay attention to the number of states for tilded deltaU, should be m - p
        {
          deltaUTilde(j, i) = uTemp[j];
        }
      }
      else
      {
        for (int j = 0; j < settings_.n_input; j++) // <-- pay attention to the number of states for tilded deltaU, should be m - p
        {
          deltaU(j, i) = uTemp[j];
        }
      }
    }

    // remap the tilde delta u to real delta u
    if (settings_.constrained && settings_.qr_decomp)
    {
      for (int i = 0; i < settings_.N; i++)
      {
        deltaU.col(i) = Q2_data[i] * deltaUTilde.col(i) - Q1_data[i] * (R1_data[i].transpose()).inverse() * (C_data[i] * deltaX.col(i) + e_data[i]);
      }
    }

    auto endFillTime = std::chrono::steady_clock::now();
    auto fillIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endFillTime - startFillTime);
    scalar_t fillTime = std::chrono::duration<scalar_t, std::milli>(fillIntervalTime).count();
    std::cout << "Retrieve time usage: " << fillTime << "[ms]." << std::endl;

    return std::make_tuple(deltaX, deltaU);
  }

  void MultipleShootingSolver::setupCostDynamicsEqualityConstraint(SystemDynamicsBase &systemDynamicsObj,
                                                                   CostFunctionBase &costFunctionObj,
                                                                   ConstraintBase &constraintObj,
                                                                   scalar_t delta_t_,
                                                                   scalar_t initTime,
                                                                   const matrix_t &x,
                                                                   const matrix_t &u,
                                                                   const vector_t &initState)
  {
    // Matrix x of shape (n_state, N+1), n = n_state;
    // Matrix u of shape (n_input, N), m = n_input;
    // Matrix pi of shape (n_state, N), this is not used temporarily
    // N is the horizon length
    // Vector initState of shape (n_state, 1)

    auto startSetupTime = std::chrono::steady_clock::now();
    scalar_t totalQRTime = 0.0;
    scalar_t totalAdditionalTime = 0.0;

    scalar_t operTime = initTime;

    // NOTE: This setup will take longer time if in debug mode
    // the most likely reason is that Eigen QR decomposition and later on calculations are not done in the optimal way in debug mode
    // everything works fine when changed to release mode
    for (int i = 0; i < settings_.N; i++)
    {
      ocs2::VectorFunctionLinearApproximation systemDynamicApprox = systemDynamicsObj.linearApproximation(operTime, x.col(i), u.col(i));
      // dx_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
      // A_{k} = Id + dt * dfdx
      // B_{k} = dt * dfdu
      // b_{k} = x_{n} + dt * f(x_{n},u_{n}) - x_{n+1}
      // currently only one-step Euler integration, subject to modification to RK4
      A_data[i] = delta_t_ * systemDynamicApprox.dfdx + matrix_t::Identity(settings_.n_state, settings_.n_state);
      B_data[i] = delta_t_ * systemDynamicApprox.dfdu;
      b_data[i] = x.col(i) + delta_t_ * systemDynamicApprox.f - x.col(i + 1);
      if (i == 0)
      {
        b_data[i] += A_data[i] * (initState - x.col(0)); // to ignore the bounding condition for x0
      }

      ocs2::ScalarFunctionQuadraticApproximation costFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(i), u.col(i));
      Q_data[i] = costFunctionApprox.dfdxx;
      R_data[i] = costFunctionApprox.dfduu;
      S_data[i] = costFunctionApprox.dfdux;
      q_data[i] = costFunctionApprox.dfdx;
      r_data[i] = costFunctionApprox.dfdu;

      if (settings_.constrained)
      {
        ocs2::VectorFunctionLinearApproximation constraintApprox = constraintObj.stateInputEqualityConstraintLinearApproximation(operTime, x.col(i), u.col(i));
        // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
        // C_{k} = constraintApprox.dfdx
        // D_{k} = constraintApprox.dfdu
        // e_{k} = constraintApprox.f
        if (settings_.qr_decomp)
        {
          // handle equality constraints using QR decomposition
          matrix_t C = constraintApprox.dfdx; // p x n
          matrix_t D = constraintApprox.dfdu; // p x m
          vector_t e = constraintApprox.f;    // p x 1
          if (i == 0)
          {
            e += C * (initState - x.col(0)); // due to x0 is not part of optimization variables
          }
          matrix_t D_transpose = D.transpose(); // m x p
          Eigen::HouseholderQR<matrix_t> qr(D_transpose);
          matrix_t Q = qr.householderQ();                            // m x m
          matrix_t R = qr.matrixQR().triangularView<Eigen::Upper>(); // m x p
          // D_transpose = Q * R
          matrix_t Q1 = Q.leftCols(settings_.n_constraint);                      // m x p
          matrix_t Q2 = Q.rightCols(settings_.n_input - settings_.n_constraint); // m x (m-p)
          // Q = [Q1, Q2]
          matrix_t R1 = R.topRows(settings_.n_constraint); // p x p
                                                           // R = [R1;
                                                           //      0]

          // store the matrices Ck, Q1k, Q2k, R1k and vector ek for later remapping \tilde{\delta uk} -> \delta uk
          C_data[i] = C;
          e_data[i] = e;
          Q1_data[i] = Q1;
          Q2_data[i] = Q2;
          R1_data[i] = R1;

          // some intermediate variables used multiple times
          matrix_t P_ke = Q1 * (R1.transpose()).inverse();
          matrix_t P_kx = P_ke * C;

          // see the doc/deduction.pdf for more details
          A_data[i] = A_data[i] - B_data[i] * P_kx;
          b_data[i] = b_data[i] - B_data[i] * P_ke * e;
          B_data[i] = B_data[i] * Q2;

          vector_t q_1 = q_data[i] - P_kx.transpose() * r_data[i];
          vector_t q_2 = -S_data[i].transpose() * P_ke * e - P_kx.transpose() * R_data[i] * P_ke * e;
          q_data[i] = q_1 + q_2;
          vector_t r_1 = Q2.transpose() * r_data[i];
          vector_t r_2 = -Q2.transpose() * R_data[i] * P_ke * e;
          r_data[i] = r_1 + r_2;
          Q_data[i] = Q_data[i] - P_kx.transpose() * S_data[i] - S_data[i].transpose() * P_kx + P_kx.transpose() * R_data[i] * P_kx;
          S_data[i] = Q2.transpose() * S_data[i] - Q2.transpose() * R_data[i] * P_kx;
          R_data[i] = Q2.transpose() * R_data[i] * Q2;
        }
        else
        {
          // for hpipm --> ug >= C*dx + D*du >= lg
          C_C_data[i] = constraintApprox.dfdx;
          D_D_data[i] = constraintApprox.dfdu;
          lg_data[i] = -constraintApprox.f;
          if (i == 0)
          {
            lg_data[i] -= C_C_data[i] * (initState - x.col(0)); // due to x0 is not part of optimization variables
          }
          ug_data[i] = lg_data[i];
        }
      }

      operTime += delta_t_;
    }

    // we should have used the finalCostQuadraticApproximation defined by Q_final matrix, just like the following:
    // ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.finalCostQuadraticApproximation(operTime, x.col(N));
    // but in this case, it is zero, which leads to unpenalized ending states
    // so I temporarily used costQuadraticApproximation with a random linearization point of input u
    ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(settings_.N), u.col(settings_.N - 1));
    Q_data[settings_.N] = 10 * finalCostFunctionApprox.dfdxx; // manually add larger penalty s.t. the final state converges to the ref state
    q_data[settings_.N] = 10 * finalCostFunctionApprox.dfdx;  // 10 is a customized number, subject to adjustment

    auto endSetupTime = std::chrono::steady_clock::now();
    auto setupIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSetupTime - startSetupTime);
    scalar_t setupTime = std::chrono::duration<scalar_t, std::milli>(setupIntervalTime).count();
    std::cout << "setup time usage: " << setupTime << "[ms]." << std::endl;
  }

  void MultipleShootingSolver::setupDimension()
  {
    nx_.resize(settings_.N + 1, settings_.n_state);
    nx_[0] = 0;

    if (settings_.constrained && settings_.qr_decomp)
    {
      nu_.resize(settings_.N + 1, settings_.n_input - settings_.n_constraint);
    }
    else
    {
      nu_.resize(settings_.N + 1, settings_.n_input);
    }
    nu_[settings_.N] = 0;

    if (settings_.constrained && !settings_.qr_decomp)
    {
      ng_.resize(settings_.N + 1, settings_.n_constraint);
      ng_[settings_.N] = 0;
    }
    else
    {
      ng_.resize(settings_.N + 1, 0);
    }

    nbu_.resize(settings_.N + 1, 0);
    nbx_.resize(settings_.N + 1, 0);
    nsbx_.resize(settings_.N + 1, 0);
    nsbu_.resize(settings_.N + 1, 0);
    nsg_.resize(settings_.N + 1, 0);

    A_data.resize(settings_.N);
    B_data.resize(settings_.N);
    b_data.resize(settings_.N);
    Q_data.resize(settings_.N + 1);
    R_data.resize(settings_.N);
    S_data.resize(settings_.N);
    q_data.resize(settings_.N + 1);
    r_data.resize(settings_.N);
    if (settings_.constrained)
    {
      if (settings_.qr_decomp)
      {
        C_data.resize(settings_.N);
        e_data.resize(settings_.N);
        Q1_data.resize(settings_.N);
        Q2_data.resize(settings_.N);
        R1_data.resize(settings_.N);
      }
      else
      {
        C_C_data.resize(settings_.N);
        D_D_data.resize(settings_.N);
        lg_data.resize(settings_.N);
        ug_data.resize(settings_.N);
      }
    }
  }

  void MultipleShootingSolver::freeHPIPMMem()
  {
    // free the hpipm memories
    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);
    dim_mem = nullptr;
    qp_mem = nullptr;
    qp_sol_mem = nullptr;
    ipm_arg_mem = nullptr;
    ipm_mem = nullptr;
  }

} // namespace ocs2