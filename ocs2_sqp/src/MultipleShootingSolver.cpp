//
// Created by rgrandia on 09.11.20.
//

#include "ocs2_sqp/MultipleShootingSolver.h"
#include <ocs2_core/control/FeedforwardController.h>
#include <iostream>

namespace ocs2
{

  MultipleShootingSolver::MultipleShootingSolver(MultipleShootingSolverSettings settings, const SystemDynamicsBaseAD *systemDynamicsPtr,
                                                 const CostFunctionBase *costFunctionPtr)
      : Solver_BASE(),
        systemDynamicsPtr_(systemDynamicsPtr->clone()),
        costFunctionPtr_(costFunctionPtr->clone()),
        settings_(std::move(settings)) {}

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
    matrix_t x = matrix_t::Random(settings_.nx, settings_.N + 1);
    matrix_t u = matrix_t::Random(settings_.nu, settings_.N);
    matrix_t pi = matrix_t::Random(settings_.nx, settings_.N);

    vector_t refState(settings_.nx);
    refState.setZero();
    refState(0) = 1.0;
    refState(1) = 1.0;
    for (int i = 0; i < settings_.sqp_iteration; i++)
    {
      std::cout << "\n---------------sqp iteration " << i << "----------\n";
      matrix_t delta_x, delta_u, new_pi;
      std::tie(delta_x, delta_u, new_pi) = runSingleIter(*systemDynamicsPtr_, *costFunctionPtr_, delta_t_, x, u, pi, initState, refState);
      x += delta_x;
      u += delta_u;
      pi = new_pi;
      // std::cout << "new x: \n";
      // std::cout << x << std::endl;
      // std::cout << "new u: \n";
      // std::cout << u << std::endl;
    }

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

  std::tuple<matrix_t, matrix_t, matrix_t> MultipleShootingSolver::runSingleIter(SystemDynamicsBaseAD &systemDynamicsPtr,
                                                                                 CostFunctionBase &costFunctionPtr,
                                                                                 scalar_t delta_t_,
                                                                                 matrix_t x,
                                                                                 matrix_t u,
                                                                                 matrix_t pi,
                                                                                 const vector_t &initState,
                                                                                 const vector_t &refState)
  {
    // Matrix x of shape (nx, N+1)
    // Matrix u of shape (nu, N)
    // Matrix pi of shape (nx, N), this is not used temporarily
    // N is the horizon length
    // Vector x_init of shape (nx, 1)

    scalar_t time_ = 0.0; //  this is used in linearization. Since our system dynamics is time-invariant, this does not matter.

    int N = u.cols();
    int n_state = x.rows();
    int n_input = u.rows();

    // number of input
    Eigen::VectorXi nnu_eigen(N + 1);
    nnu_eigen.setOnes();
    nnu_eigen *= n_input;
    nnu_eigen(N) = 0;
    int *nnu = nnu_eigen.data();

    // number of states
    Eigen::VectorXi nnx_eigen(N + 1);
    nnx_eigen.setOnes();
    nnx_eigen *= n_state;
    int *nnx = nnx_eigen.data();

    // number of input box constraints
    Eigen::VectorXi nnbu_eigen(N + 1);
    nnbu_eigen.setZero();
    int *nnbu = nnbu_eigen.data();

    // number of states box constraints
    Eigen::VectorXi nnbx_eigen(N + 1);
    nnbx_eigen.setZero();
    nnbx_eigen(0) = n_state;
    int *nnbx = nnbx_eigen.data();

    // number of general constraints
    Eigen::VectorXi nng_eigen(N + 1);
    nng_eigen.setZero();
    int *nng = nng_eigen.data();

    // number of softed constraints on state box constraints
    Eigen::VectorXi nnsbx_eigen(N + 1);
    nnsbx_eigen.setZero();
    int *nnsbx = nnsbx_eigen.data();

    // number of softed constraints on input box constraints
    Eigen::VectorXi nnsbu_eigen(N + 1);
    nnsbu_eigen.setZero();
    int *nnsbu = nnsbu_eigen.data();

    // number of softed constraints on general constraints
    Eigen::VectorXi nnsg_eigen(N + 1);
    nnsg_eigen.setZero();
    int *nnsg = nnsg_eigen.data();

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

    for (int i = 0; i < N; i++)
    {
      ocs2::VectorFunctionLinearApproximation derivative_result = systemDynamicsPtr.linearApproximation(time_, x.col(i), u.col(i));
      A_data[i] = delta_t_ * derivative_result.dfdx + matrix_t::Identity(n_state, n_state);
      AA[i] = A_data[i].data();
      B_data[i] = delta_t_ * derivative_result.dfdu;
      BB[i] = B_data[i].data();
      b_data[i] = x.col(i) + delta_t_ * derivative_result.f - x.col(i + 1);
      bb[i] = b_data[i].data();

      ocs2::ScalarFunctionQuadraticApproximation costfunction_approx = costFunctionPtr.costQuadraticApproximation(time_, x.col(i), u.col(i));
      Q_data[i] = delta_t_ * costfunction_approx.dfdxx;
      QQ[i] = Q_data[i].data();
      R_data[i] = delta_t_ * costfunction_approx.dfduu;
      RR[i] = R_data[i].data();
      S_data[i] = delta_t_ * costfunction_approx.dfdux;
      SS[i] = S_data[i].data();
      q_data[i] = delta_t_ * (costfunction_approx.dfdx - costfunction_approx.dfdxx * refState);
      qq[i] = q_data[i].data();
      r_data[i] = delta_t_ * costfunction_approx.dfdu;
      rr[i] = r_data[i].data();
    }
    ocs2::ScalarFunctionQuadraticApproximation costfunction_approx = costFunctionPtr.costQuadraticApproximation(time_, x.col(N), u.col(N - 1));
    Q_data[N] = delta_t_ * costfunction_approx.dfdxx;
    q_data[N] = delta_t_ * costfunction_approx.dfdx;
    QQ[N] = Q_data[N].data();
    qq[N] = q_data[N].data();

    vector_t init_l = initState - x.col(0);
    vector_t init_u = initState - x.col(0);

    scalar_t *lbx0 = init_l.data();
    scalar_t *ubx0 = init_u.data();

    Eigen::VectorXi idxbx0_const(n_state);
    idxbx0_const << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
    int *idxbx0 = idxbx0_const.data();

    vector_t u_guess_eigen(n_input);
    u_guess_eigen.setZero();
    scalar_t *u_guess = u_guess_eigen.data();

    vector_t x_guess_eigen(n_state);
    x_guess_eigen.setZero();
    scalar_t *x_guess = x_guess_eigen.data();

    scalar_t sl_guess[] = {};
    scalar_t su_guess[] = {};

    int *iidxbx[N + 1];
    scalar_t *llbx[N + 1];
    scalar_t *uubx[N + 1];
    iidxbx[0] = idxbx0;
    llbx[0] = lbx0;
    uubx[0] = ubx0;
    for (int i = 1; i < N + 1; i++)
    {
      iidxbx[i] = NULL;
      llbx[i] = NULL;
      uubx[i] = NULL;
    }

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

    scalar_t *uu_guess[N + 1];
    scalar_t *xx_guess[N + 1];
    scalar_t *ssl_guess[N + 1];
    scalar_t *ssu_guess[N + 1];
    for (int i = 0; i < N + 1; i++)
    {
      uu_guess[i] = u_guess;
      xx_guess[i] = x_guess;
      ssl_guess[i] = sl_guess;
      ssu_guess[i] = su_guess;
    }

    int *nu = nnu;
    int *nx = nnx;
    int *nbu = nnbu;
    int *nbx = nnbx;
    int *ng = nng;
    int *nsbx = nnsbx;
    int *nsbu = nnsbu;
    int *nsg = nnsg;

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

    int ii, jj;

    int hpipm_status;

    int rep;
    int nrep = 10;

    struct timeval tv0, tv1;

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

    hpipm_timer timer;
    hpipm_tic(&timer);

    for (rep = 0; rep < nrep; rep++)
    {
      // call solver
      d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
      d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
    }

    scalar_t time_ipm = hpipm_toc(&timer) / nrep;

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

    // int iter;
    // d_ocp_qp_ipm_get_iter(&workspace, &iter);
    // scalar_t res_stat;
    // d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
    // scalar_t res_eq;
    // d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
    // scalar_t res_ineq;
    // d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
    // scalar_t res_comp;
    // d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
    // scalar_t *stat;
    // d_ocp_qp_ipm_get_stat(&workspace, &stat);
    // int stat_m;
    // d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

    // printf("\nipm return = %d\n", hpipm_status);
    // printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

    // printf("\nipm iter = %d\n", iter);
    // printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
    // d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);

    // printf("\nocp ipm time = %e [s]\n\n", time_ipm);

    matrix_t u_sol(n_input, N);
    matrix_t x_sol(n_state, N + 1);
    matrix_t pi_sol(n_state, N);
    scalar_t u_carray[n_input];
    for (ii = 0; ii < N; ii++)
    {
      d_ocp_qp_sol_get_u(ii, &qp_sol, u_carray);
      for (int j = 0; j < n_input; j++)
      {
        u_sol(j, ii) = u_carray[j];
      }
    }

    scalar_t x_carray[n_state];
    for (ii = 0; ii <= N; ii++)
    {
      d_ocp_qp_sol_get_x(ii, &qp_sol, x_carray);
      for (int j = 0; j < n_state; j++)
      {
        x_sol(j, ii) = x_carray[j];
      }
    }

    scalar_t pi_carray[n_state];
    for (ii = 0; ii < N; ii++)
    {
      d_ocp_qp_sol_get_pi(ii, &qp_sol, pi_carray);
      for (int j = 0; j < n_state; j++)
      {
        pi_sol(j, ii) = pi_carray[j];
      }
    }
    // printf("\ndelta u = \n");
    // std::cout << u_sol << std::endl;
    // printf("\ndelta x = \n");
    // std::cout << x_sol << std::endl;
    // printf("\npi = \n");
    // std::cout << pi_sol << std::endl;
    printf("one iteration ends. \n");

    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);

    return std::make_tuple(x_sol, u_sol, pi_sol);
  }

} // namespace ocs2