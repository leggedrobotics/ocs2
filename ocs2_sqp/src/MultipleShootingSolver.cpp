//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <iostream>
#include <chrono>

namespace ocs2
{

  MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBaseAD *systemDynamicsPtr,
                                                 const CostFunctionBase *costFunctionPtr)
      : Solver_BASE(),
        systemDynamicsPtr_(systemDynamicsPtr->clone()),
        costFunctionPtr_(costFunctionPtr->clone()),
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

  void MultipleShootingSolver::runImpl(scalar_t initTime, const vector_t &initState, scalar_t finalTime,
                                       const scalar_array_t &partitioningTimes)
  {
    // ignore partitioningTimes

    // Initialize cost
    costFunctionPtr_->setCostDesiredTrajectoriesPtr(&this->getCostDesiredTrajectories());

    // Solve the problem.
    scalar_t delta_t_ = (finalTime - initTime) / settings_.N;
    matrix_t x(settings_.nx, settings_.N + 1);
    matrix_t u(settings_.nu, settings_.N);
    if (!settings_.initPrimalSol)
    {
      x = matrix_t::Random(settings_.nx, settings_.N + 1);
      u = matrix_t::Random(settings_.nu, settings_.N);
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
      matrix_t delta_x, delta_u;
      std::tie(delta_x, delta_u) = runSingleIter(*systemDynamicsPtr_, *costFunctionPtr_, delta_t_, initTime, x, u, initState);
      x += delta_x;
      u += delta_u;
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

  std::tuple<matrix_t, matrix_t> MultipleShootingSolver::runSingleIter(SystemDynamicsBaseAD &systemDynamicsObj,
                                                                       CostFunctionBase &costFunctionObj,
                                                                       scalar_t delta_t_,
                                                                       scalar_t initTime,
                                                                       const matrix_t &x,
                                                                       const matrix_t &u,
                                                                       const vector_t &initState)
  {
    // Matrix x of shape (nx, N+1)
    // Matrix u of shape (nu, N)
    // Matrix pi of shape (nx, N), this is not used temporarily
    // N is the horizon length
    // Vector x_init of shape (nx, 1)

    auto startAllocTime = std::chrono::steady_clock::now();
    int N = u.cols();
    int n_state = x.rows();
    int n_input = u.rows();

    // number of input
    int nnuData[N + 1];
    // number of states
    int nnxData[N + 1];
    // number of input box constraints
    int nnbuData[N + 1];
    // number of states box constraints
    int nnbxData[N + 1];
    // number of general constraints
    int nngData[N + 1];
    // number of softed constraints on state box constraints
    int nnsbxData[N + 1];
    // number of softed constraints on input box constraints
    int nnsbuData[N + 1];
    // number of softed constraints on general constraints
    int nnsgData[N + 1];

    int *iidxbx[N + 1];
    scalar_t *llbx[N + 1];
    scalar_t *uubx[N + 1];

    scalar_t *uu_guess[N + 1];
    scalar_t *xx_guess[N + 1];
    scalar_t *ssl_guess[N + 1];
    scalar_t *ssu_guess[N + 1];

    vector_t init_l = initState - x.col(0);
    vector_t init_u = initState - x.col(0);
    scalar_t *lbx0 = init_l.data();
    scalar_t *ubx0 = init_u.data();

    int idxbx0Data[n_state];
    scalar_t x_guessData[n_state];
    scalar_t u_guessData[n_input];
    for (int i = 0; i < n_state; i++)
    {
      x_guessData[i] = 0.0;
      idxbx0Data[i] = i;
    }
    for (int i = 0; i < n_input; i++)
    {
      u_guessData[i] = 0.0;
    }

    int *idxbx0 = idxbx0Data;
    scalar_t *x_guess = x_guessData;
    scalar_t *u_guess = u_guessData;
    scalar_t sl_guess[] = {};
    scalar_t su_guess[] = {};

    for (int i = 0; i < N + 1; i++)
    {
      nnuData[i] = n_input;
      nnxData[i] = n_state;
      nnbuData[i] = 0;
      nnbxData[i] = 0;
      nngData[i] = 0;
      nnsbxData[i] = 0;
      nnsbuData[i] = 0;
      nnsgData[i] = 0;
      iidxbx[i] = NULL;
      llbx[i] = NULL;
      uubx[i] = NULL;
      uu_guess[i] = u_guess;
      xx_guess[i] = x_guess;
      ssl_guess[i] = sl_guess;
      ssu_guess[i] = su_guess;
    }
    nnuData[N] = 0;
    nnbxData[0] = n_state;
    iidxbx[0] = idxbx0;
    llbx[0] = lbx0;
    uubx[0] = ubx0;
    int *nu = nnuData;
    int *nx = nnxData;
    int *nbu = nnbuData;
    int *nbx = nnbxData;
    int *ng = nngData;
    int *nsbx = nnsbxData;
    int *nsbu = nnsbuData;
    int *nsg = nnsgData;

    scalar_t *AA[N + 1];
    scalar_t *BB[N];
    scalar_t *bb[N];
    scalar_t *QQ[N + 1];
    scalar_t *RR[N];
    scalar_t *SS[N];
    scalar_t *qq[N + 1];
    scalar_t *rr[N];

    matrix_array_t A_data;
    matrix_array_t B_data;
    matrix_array_t b_data;
    matrix_array_t Q_data;
    matrix_array_t R_data;
    matrix_array_t S_data;
    matrix_array_t q_data;
    matrix_array_t r_data;
    A_data.resize(N);
    B_data.resize(N);
    b_data.resize(N);
    Q_data.resize(N + 1);
    R_data.resize(N);
    S_data.resize(N);
    q_data.resize(N + 1);
    r_data.resize(N);

    scalar_t operTime = initTime;

    for (int i = 0; i < N; i++)
    {
      ocs2::VectorFunctionLinearApproximation systemDynamicApprox = systemDynamicsObj.linearApproximation(operTime, x.col(i), u.col(i));
      A_data[i] = delta_t_ * systemDynamicApprox.dfdx + matrix_t::Identity(n_state, n_state);
      AA[i] = A_data[i].data();
      B_data[i] = delta_t_ * systemDynamicApprox.dfdu;
      BB[i] = B_data[i].data();
      b_data[i] = x.col(i) + delta_t_ * systemDynamicApprox.f - x.col(i + 1);
      bb[i] = b_data[i].data();

      ocs2::ScalarFunctionQuadraticApproximation costFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(i), u.col(i));
      Q_data[i] = delta_t_ * costFunctionApprox.dfdxx;
      QQ[i] = Q_data[i].data();
      R_data[i] = delta_t_ * costFunctionApprox.dfduu;
      RR[i] = R_data[i].data();
      S_data[i] = delta_t_ * costFunctionApprox.dfdux;
      SS[i] = S_data[i].data();
      q_data[i] = delta_t_ * costFunctionApprox.dfdx;
      qq[i] = q_data[i].data();
      r_data[i] = delta_t_ * costFunctionApprox.dfdu;
      rr[i] = r_data[i].data();

      operTime += delta_t_;
    }

    // we should have used the finalCostQuadraticApproximation defined by Q_final matrix, but in this case, it is zero, which leads to diverging ending states
    // so I temporarily used costQuadraticApproximation with a random linearization point of input u
    // ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.finalCostQuadraticApproximation(operTime, x.col(N));
    ocs2::ScalarFunctionQuadraticApproximation finalCostFunctionApprox = costFunctionObj.costQuadraticApproximation(operTime, x.col(N), u.col(N - 1));
    Q_data[N] = 10 * delta_t_ * finalCostFunctionApprox.dfdxx; // manually add larger penalty s.t. the final state converges to the ref state
    q_data[N] = 10 * delta_t_ * finalCostFunctionApprox.dfdx;
    QQ[N] = Q_data[N].data();
    qq[N] = q_data[N].data();

    int *iidxbu[N + 1] = {};
    scalar_t *llbu[N + 1] = {};
    scalar_t *uubu[N + 1] = {};
    scalar_t *CC[N + 1] = {};
    scalar_t *DD[N + 1] = {};
    scalar_t *llg[N + 1] = {};
    scalar_t *uug[N + 1] = {};
    scalar_t *ZZl[N + 1] = {};
    scalar_t *ZZu[N + 1] = {};
    scalar_t *zzl[N + 1] = {};
    scalar_t *zzu[N + 1] = {};
    int *iidxs[N + 1] = {};
    scalar_t *llls[N + 1] = {};
    scalar_t *llus[N + 1] = {};

    scalar_t **hA = AA;
    scalar_t **hB = BB;
    scalar_t **hb = bb;
    scalar_t **hQ = QQ;
    scalar_t **hR = RR;
    scalar_t **hS = SS;
    scalar_t **hq = qq;
    scalar_t **hr = rr;
    int **hidxbx = iidxbx;
    scalar_t **hlbx = llbx;
    scalar_t **hubx = uubx;
    int **hidxbu = iidxbu;
    scalar_t **hlbu = llbu;
    scalar_t **hubu = uubu;
    scalar_t **hC = CC;
    scalar_t **hD = DD;
    scalar_t **hlg = llg;
    scalar_t **hug = uug;
    scalar_t **hZl = ZZl;
    scalar_t **hZu = ZZu;
    scalar_t **hzl = zzl;
    scalar_t **hzu = zzu;
    int **hidxs = iidxs;
    scalar_t **hlls = llls;
    scalar_t **hlus = llus;

    scalar_t **hu_guess = uu_guess;
    scalar_t **hx_guess = xx_guess;
    scalar_t **hsl_guess = ssl_guess;
    scalar_t **hsu_guess = ssu_guess;

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

    int hpipm_status;
    int nrep = 10;

    int dim_size = d_ocp_qp_dim_memsize(N);
    void *dim_mem = malloc(dim_size);

    struct d_ocp_qp_dim dim;
    d_ocp_qp_dim_create(N, &dim, dim_mem);

    d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

    int qp_size = d_ocp_qp_memsize(&dim);
    void *qp_mem = malloc(qp_size);

    struct d_ocp_qp qp;
    d_ocp_qp_create(&dim, &qp, qp_mem);

    d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

    int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    void *qp_sol_mem = malloc(qp_sol_size);

    struct d_ocp_qp_sol qp_sol;
    d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    void *ipm_arg_mem = malloc(ipm_arg_size);

    struct d_ocp_qp_ipm_arg arg;
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

    ::hpipm_mode mode = ::hpipm_mode::SPEED;

    d_ocp_qp_ipm_arg_set_default(mode, &arg);

    d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
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
    void *ipm_mem = malloc(ipm_size);

    struct d_ocp_qp_ipm_ws workspace;
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    auto endAllocTime = std::chrono::steady_clock::now();
    auto allocIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endAllocTime - startAllocTime);
    scalar_t allocTime = std::chrono::duration<scalar_t, std::milli>(allocIntervalTime).count();
    std::cout << "Alloc time usage: " << allocTime << "[ms]." << std::endl;

    auto startSolveTime = std::chrono::steady_clock::now();

    hpipm_timer timer;
    hpipm_tic(&timer);
    for (int rep = 0; rep < nrep; rep++)
    {
      d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
      d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
    }
    scalar_t time_ipm = hpipm_toc(&timer) / nrep;

    auto endSolveTime = std::chrono::steady_clock::now();
    auto solveIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endSolveTime - startSolveTime);
    scalar_t solveTime = std::chrono::duration<scalar_t, std::milli>(solveIntervalTime).count();
    std::cout << "Solve time usage: " << solveTime << "[ms]." << std::endl;

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
      printf("\nAverage solution time over %i runs: %e [s]\n", nrep, time_ipm);
      printf("\n\n");
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

    // getting deltaX, deltaU
    auto startFillTime = std::chrono::steady_clock::now();
    matrix_t deltaU(n_input, N);
    matrix_t deltaX(n_state, N + 1);
    scalar_t uTemp[n_input];
    scalar_t xTemp[n_state];

    for (int ii = 0; ii < N; ii++)
    {
      d_ocp_qp_sol_get_x(ii, &qp_sol, xTemp);
      d_ocp_qp_sol_get_u(ii, &qp_sol, uTemp);
      for (int j = 0; j < n_input; j++)
      {
        deltaU(j, ii) = uTemp[j];
      }
      for (int j = 0; j < n_state; j++)
      {
        deltaX(j, ii) = xTemp[j];
      }
    }
    d_ocp_qp_sol_get_x(N, &qp_sol, xTemp);
    for (int j = 0; j < n_state; j++)
    {
      deltaX(j, N) = xTemp[j];
    }
    auto endFillTime = std::chrono::steady_clock::now();
    auto fillIntervalTime = std::chrono::duration_cast<std::chrono::nanoseconds>(endFillTime - startFillTime);
    scalar_t fillTime = std::chrono::duration<scalar_t, std::milli>(fillIntervalTime).count();
    std::cout << "Fill time usage: " << fillTime << "[ms]." << std::endl;

    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);

    return std::make_tuple(deltaX, deltaU);
  }

} // namespace ocs2