#include <stdlib.h>
#include <stdio.h>
#include <tuple>
#include <sys/time.h>

#include <blasfeo_d_aux_ext_dep.h>

extern "C"
{
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}

#include <Eigen/Eigen>
#include <iostream>

#include <ros/init.h>

#include "ocs2_ballbot_example/BallbotInterface.h"
#include "ocs2_ballbot_example/dynamics/BallbotSystemDynamics.h"
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>

double EPSILON = 1e-7;

void rounding(Eigen::MatrixXd &input)
{
	for (int i = 0; i < input.rows(); i++)
	{
		for (int j = 0; j < input.cols(); j++)
		{
			if (std::abs(input(i, j)) < EPSILON)
			{
				input(i, j) = 0.0;
			}
		}
	}
}

std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> qp_solution(ocs2::ballbot::BallbotSystemDynamics ballbotDynamics,
																		  ocs2::QuadraticCostFunction ballbotCostFunction,
																		  double delta_t_,
																		  Eigen::MatrixXd x, Eigen::MatrixXd u, Eigen::MatrixXd pi, Eigen::VectorXd x_init)
{
	// Matrix x of shape (nx, N+1)
	// Matrix u of shape (nu, N)
	// Matrix pi of shape (nx, N), this is not used temporarily
	// N is the horizon length
	// Vector x_init of shape (nx, 1)

	double time_ = 0.0; //  this is used in linearization. Since our system dynamics is time-invariant, this does not matter.

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

	double *AA[N + 1];
	double *BB[N];
	double *bb[N];
	double *QQ[N + 1];
	double *RR[N];
	double *SS[N];
	double *qq[N + 1];
	double *rr[N];

	Eigen::MatrixXd Q_eigen(n_state, n_state);
	Q_eigen.setIdentity();
	// Q_eigen(0, 0) = 110;
	// Q_eigen(1, 1) = 110;
	// Q_eigen(2, 2) = 400;
	// Q_eigen(3, 3) = 0;
	// Q_eigen(4, 4) = 0;
	// Q_eigen(5, 5) = 35;
	// Q_eigen(6, 6) = 35;
	// Q_eigen(7, 7) = 30;
	// Q_eigen(8, 8) = 20;
	// Q_eigen(9, 9) = 20;
	// Q_eigen *= delta_t_;

	Eigen::MatrixXd R_eigen(n_input, n_input);
	R_eigen.setIdentity();
	// R_eigen(0, 0) = 2;
	// R_eigen(1, 1) = 2;
	// R_eigen(2, 2) = 2;
	// R_eigen *= delta_t_;

	Eigen::MatrixXd S_eigen(n_input, n_state);
	S_eigen.setZero();
	// S_eigen *= delta_t_;

	for (int i = 0; i < N; i++)
	{
		ocs2::VectorFunctionLinearApproximation derivative_result = ballbotDynamics.linearApproximation(time_, x.col(i), u.col(i));
		Eigen::MatrixXd A_temp = delta_t_ * derivative_result.dfdx + Eigen::MatrixXd::Identity(n_state, n_state);
		AA[i] = A_temp.data();
		Eigen::MatrixXd B_temp = delta_t_ * derivative_result.dfdu;
		BB[i] = B_temp.data();
		Eigen::VectorXd b_temp = x.col(i) + delta_t_ * derivative_result.f - x.col(i + 1);
		bb[i] = b_temp.data();

		// ocs2::ScalarFunctionQuadraticApproximation costfunction_approx = ballbotCostFunction.costQuadraticApproximation(time_, x.col(i), u.col(i));
		// std::cout << costfunction_approx.dfdxx << std::endl;
		QQ[i] = Q_eigen.data(); // --> delta_t * dfdxx
		RR[i] = R_eigen.data(); //
		SS[i] = S_eigen.data();
		Eigen::VectorXd q_temp = Q_eigen * x.col(i);
		qq[i] = q_temp.data();
		Eigen::VectorXd r_temp = R_eigen * u.col(i);
		rr[i] = r_temp.data();
	}
	// Q_eigen.setZero();
	QQ[N] = Q_eigen.data();
	Eigen::VectorXd qN_temp = Q_eigen * x.col(N);
	qq[N] = qN_temp.data();

	Eigen::VectorXd init_temp_l = x_init - x.col(0);
	Eigen::VectorXd init_temp_u = x_init - x.col(0);

	double *lbx0 = init_temp_l.data();
	double *ubx0 = init_temp_u.data();

	Eigen::VectorXi idxbx0_const(n_state);
	idxbx0_const << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
	int *idxbx0 = idxbx0_const.data();

	Eigen::VectorXd u_guess_eigen(n_input);
	u_guess_eigen.setZero();
	double *u_guess = u_guess_eigen.data();

	Eigen::VectorXd x_guess_eigen(n_state);
	x_guess_eigen.setZero();
	double *x_guess = x_guess_eigen.data();

	double sl_guess[] = {};
	double su_guess[] = {};

	int *iidxbx[N + 1];
	double *llbx[N + 1];
	double *uubx[N + 1];
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
	double *llbu[N + 1] = {};
	double *uubu[N + 1] = {};
	double *CC[N + 1] = {};
	double *DD[N + 1] = {};
	double *llg[N + 1] = {};
	double *uug[N + 1] = {};
	double *ZZl[N + 1] = {};
	double *ZZu[N + 1] = {};
	double *zzl[N + 1] = {};
	double *zzu[N + 1] = {};
	int *iidxs[N + 1] = {};
	double *llls[N + 1] = {};
	double *llus[N + 1] = {};

	double *uu_guess[N + 1];
	double *xx_guess[N + 1];
	double *ssl_guess[N + 1];
	double *ssu_guess[N + 1];
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

	double **hA = AA;
	double **hB = BB;
	double **hb = bb;
	double **hQ = QQ;
	double **hR = RR;
	double **hS = SS;
	double **hq = qq;
	double **hr = rr;
	int **hidxbx = iidxbx;
	double **hlbx = llbx;
	double **hubx = uubx;
	int **hidxbu = iidxbu;
	double **hlbu = llbu;
	double **hubu = uubu;
	double **hC = CC;
	double **hD = DD;
	double **hlg = llg;
	double **hug = uug;
	double **hZl = ZZl;
	double **hZu = ZZu;
	double **hzl = zzl;
	double **hzu = zzu;
	int **hidxs = iidxs;
	double **hlls = llls;
	double **hlus = llus;

	double **hu_guess = uu_guess;
	double **hx_guess = xx_guess;
	double **hsl_guess = ssl_guess;
	double **hsu_guess = ssu_guess;

	int iter_max = 30;
	double alpha_min = 1e-8;
	double mu0 = 1e4;
	double tol_stat = 1e-5;
	double tol_eq = 1e-5;
	double tol_ineq = 1e-5;
	double tol_comp = 1e-5;
	double reg_prim = 1e-12;
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

	::hpipm_mode mode = ::hpipm_mode::ROBUST;

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

	double time_ipm = hpipm_toc(&timer) / nrep;

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
	else if (hpipm_status == 2)
	{
		printf("\n -> Solver failed! NaN in computations\n");
	}
	else
	{
		printf("\n -> Solver failed! Unknown return flag\n");
	}
	printf("\nAverage solution time over %i runs: %e [s]\n", nrep, time_ipm);
	printf("\n\n");

	int iter;
	d_ocp_qp_ipm_get_iter(&workspace, &iter);
	double res_stat;
	d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq;
	d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq;
	d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp;
	d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat;
	d_ocp_qp_ipm_get_stat(&workspace, &stat);
	int stat_m;
	d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);

	printf("\nocp ipm time = %e [s]\n\n", time_ipm);

	Eigen::MatrixXd u_sol(n_input, N);
	Eigen::MatrixXd x_sol(n_state, N + 1);
	Eigen::MatrixXd pi_sol(n_state, N);
	printf("\ndelta u = \n");
	double u_carray[n_input];
	for (ii = 0; ii < N; ii++)
	{
		d_ocp_qp_sol_get_u(ii, &qp_sol, u_carray);
		for (int j = 0; j < n_input; j++)
		{
			u_sol(j, ii) = u_carray[j];
		}
	}
	std::cout << u_sol << std::endl;

	printf("\ndelta x = \n");
	double x_carray[n_state];
	for (ii = 0; ii <= N; ii++)
	{
		d_ocp_qp_sol_get_x(ii, &qp_sol, x_carray);
		for (int j = 0; j < n_state; j++)
		{
			x_sol(j, ii) = x_carray[j];
		}
	}
	std::cout << x_sol << std::endl;

	printf("\npi = \n");
	double pi_carray[n_state];
	for (ii = 0; ii < N; ii++)
	{
		d_ocp_qp_sol_get_pi(ii, &qp_sol, pi_carray);
		for (int j = 0; j < n_state; j++)
		{
			pi_sol(j, ii) = pi_carray[j];
		}
	}

	std::cout << pi_sol << std::endl;
	printf("one iteration ends. \n");

	free(dim_mem);
	free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem);

	return std::make_tuple(x_sol, u_sol, pi_sol);
}

int main(int argc, char **argv)
{

	const std::string robotName = "ballbot";
	// task file
	std::vector<std::string> programArgs{};
	::ros::removeROSArgs(argc, argv, programArgs);
	if (programArgs.size() <= 1)
	{
		throw std::runtime_error("No task file specified. Aborting.");
	}
	std::string taskFileFolderName = std::string(programArgs[1]);

	// Robot interface
	ocs2::ballbot::BallbotInterface ballbotInterface(taskFileFolderName);
	ocs2::ballbot::BallbotSystemDynamics ballbotDynamics = ballbotInterface.getDynamics();
	ocs2::QuadraticCostFunction ballbotCostFunction = ballbotInterface.getCost();

	int nx = 10;			 // number of state x
	int nu = 3;				 // number of input u
	int N = 10;				 // number of horizon
	double total_time = 0.5; // in seconds
	double delta_t_ = total_time / N;

	// desired initial state
	Eigen::VectorXd x_init(nx);
	x_init << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

	std::cout << "random initial state" << std::endl;
	Eigen::MatrixXd x = Eigen::MatrixXd::Random(nx, N + 1);
	std::cout << x << std::endl;
	std::cout << "random initial input" << std::endl;
	Eigen::MatrixXd u = Eigen::MatrixXd::Random(nu, N);
	std::cout << u << std::endl;
	Eigen::MatrixXd pi = Eigen::MatrixXd::Random(nx, N);

	int sqp_iteration = 10;
	for (int i = 0; i < sqp_iteration; i++)
	{
		std::cout << "\n---------------sqp iteration " << i << "----------\n";
		Eigen::MatrixXd delta_x, delta_u, new_pi;
		std::tie(delta_x, delta_u, new_pi) = qp_solution(ballbotDynamics, ballbotCostFunction, delta_t_, x, u, pi, x_init);
		x += delta_x;
		u += delta_u;
		pi = new_pi;
		std::cout << "new x" << std::endl;
		std::cout << x << std::endl;
		std::cout << "new u" << std::endl;
		std::cout << u << std::endl;
	}

	return 0;
}
