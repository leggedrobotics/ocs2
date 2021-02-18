//
// Created by rgrandia on 18.02.21.
//

#include "ocs2_sqp/HpipmInterface.h"

#include <blasfeo_d_aux_ext_dep.h>
extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}

namespace ocs2 {

class HpipmInterface::Impl {
 public:
  Impl(OcpSize ocpSize, Settings settings) : ocpSize_(std::move(ocpSize)) {
    ocpSize_.nx[0] = 0;

    const int dim_size = d_ocp_qp_dim_memsize(ocpSize_.N);
    dim_mem = malloc(dim_size);
    d_ocp_qp_dim_create(ocpSize_.N, &dim, dim_mem);
    d_ocp_qp_dim_set_all(ocpSize_.nx.data(), ocpSize_.nu.data(), ocpSize_.nbx.data(), ocpSize_.nbu.data(), ocpSize_.ng.data(),
                         ocpSize_.nsbx.data(), ocpSize_.nsbu.data(), ocpSize_.nsg.data(), &dim);

    const int qp_size = d_ocp_qp_memsize(&dim);
    qp_mem = malloc(qp_size);
    d_ocp_qp_create(&dim, &qp, qp_mem);

    const int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
    qp_sol_mem = malloc(qp_sol_size);
    d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    const int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
    ipm_arg_mem = malloc(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
    d_ocp_qp_ipm_arg_set_default(mode, &arg);

    const int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
    ipm_mem = malloc(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    applySettings(settings);
  }

  ~Impl() {
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

  void applySettings(Settings& settings) {
    d_ocp_qp_ipm_arg_set_iter_max(&settings.iter_max, &arg);
    d_ocp_qp_ipm_arg_set_alpha_min(&settings.alpha_min, &arg);
    d_ocp_qp_ipm_arg_set_mu0(&settings.mu0, &arg);
    d_ocp_qp_ipm_arg_set_tol_stat(&settings.tol_stat, &arg);
    d_ocp_qp_ipm_arg_set_tol_eq(&settings.tol_eq, &arg);
    d_ocp_qp_ipm_arg_set_tol_ineq(&settings.tol_ineq, &arg);
    d_ocp_qp_ipm_arg_set_tol_comp(&settings.tol_comp, &arg);
    d_ocp_qp_ipm_arg_set_reg_prim(&settings.reg_prim, &arg);
    d_ocp_qp_ipm_arg_set_warm_start(&settings.warm_start, &arg);
    d_ocp_qp_ipm_arg_set_pred_corr(&settings.pred_corr, &arg);
    d_ocp_qp_ipm_arg_set_ric_alg(&settings.ric_alg, &arg);
  }

  void solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
             std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& stateTrajectory,
             std::vector<vector_t>& inputTrajectory, bool verbose) {
    assert(dynamics.size() == ocpSize_.N);
    assert(cost.size() == (ocpSize_.N + 1));
    // TODO: check state input size.

    std::vector<scalar_t*> AA(ocpSize_.N);
    std::vector<scalar_t*> BB(ocpSize_.N);
    std::vector<scalar_t*> bb(ocpSize_.N);
    std::vector<scalar_t*> QQ(ocpSize_.N + 1);
    std::vector<scalar_t*> RR(ocpSize_.N);
    std::vector<scalar_t*> SS(ocpSize_.N);
    std::vector<scalar_t*> qq(ocpSize_.N + 1);
    std::vector<scalar_t*> rr(ocpSize_.N);

    // Dynamics k = 0
    vector_t b0 = dynamics[0].f;
    b0.noalias() += dynamics[0].dfdx * x0;
    AA[0] = nullptr;
    BB[0] = dynamics[0].dfdu.data();
    bb[0] = b0.data();

    // Dynamics k = 1 -> N-1
    for (int k = 1; k < ocpSize_.N; k++) {
      AA[k] = dynamics[k].dfdx.data();
      BB[k] = dynamics[k].dfdu.data();
      bb[k] = dynamics[k].f.data();
    }

    // Cost k = 0 -> N
    for (int k = 0; k < (ocpSize_.N + 1); k++) {
      QQ[k] = cost[k].dfdxx.data();
      RR[k] = cost[k].dfduu.data();
      SS[k] = cost[k].dfdux.data();
      qq[k] = cost[k].dfdx.data();
      rr[k] = cost[k].dfdu.data();
    }

    int** hidxbx = nullptr;
    scalar_t** hlbx = nullptr;
    scalar_t** hubx = nullptr;
    int** hidxbu = nullptr;
    scalar_t** hlbu = nullptr;
    scalar_t** hubu = nullptr;
    scalar_t** hC = nullptr;
    scalar_t** hD = nullptr;
    scalar_t** hlg = nullptr;
    scalar_t** hug = nullptr;
    scalar_t** hZl = nullptr;
    scalar_t** hZu = nullptr;
    scalar_t** hzl = nullptr;
    scalar_t** hzu = nullptr;
    int** hidxs = nullptr;
    scalar_t** hlls = nullptr;
    scalar_t** hlus = nullptr;
    d_ocp_qp_set_all(AA.data(), BB.data(), bb.data(), QQ.data(), SS.data(), RR.data(), qq.data(), rr.data(), hidxbx, hlbx, hubx, hidxbu,
                     hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

    d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
    d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);

    if (verbose) {
      printStatus();
    }

    getStateSolution(x0, stateTrajectory);
    getInputSolution(inputTrajectory);
  }

  void getStateSolution(const vector_t& x0, std::vector<vector_t>& stateTrajectory) {
    stateTrajectory.resize(ocpSize_.N + 1);
    stateTrajectory.front() = x0;
    for (int k = 1; k < (ocpSize_.N + 1); ++k) {
      d_ocp_qp_sol_get_x(k, &qp_sol, stateTrajectory[k].data());
    }
  }

  void getInputSolution(std::vector<vector_t>& inputTrajectory) {
    inputTrajectory.resize(ocpSize_.N);
    for (int k = 0; k < (ocpSize_.N); ++k) {
      d_ocp_qp_sol_get_u(k, &qp_sol, inputTrajectory[k].data());
    }
  }

  void printStatus() {
    printf("\nHPIPM returned with flag %i.\n", hpipm_status);
    if (hpipm_status == 0) {
      printf("\n -> QP solved!\n");
    } else if (hpipm_status == 1) {
      printf("\n -> Solver failed! Maximum number of iterations reached\n");
    } else if (hpipm_status == 2) {
      printf("\n -> Solver failed! Minimum step length reached\n");
    } else if (hpipm_status == 3) {
      printf("\n -> Solver failed! NaN in computations\n");
    } else {
      printf("\n -> Solver failed! Unknown return flag\n");
    }

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
    scalar_t* stat;
    d_ocp_qp_ipm_get_stat(&workspace, &stat);
    int stat_m;
    d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);
    printf("\nipm return = %d\n", hpipm_status);
    printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);
    printf("\nipm iter = %d\n", iter);
    printf(
        "\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref "
        "pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
    d_print_exp_tran_mat(stat_m, iter + 1, stat, stat_m);
  }

 private:
  OcpSize ocpSize_;

  void* dim_mem;
  d_ocp_qp_dim dim;

  void* qp_mem;
  d_ocp_qp qp;

  void* qp_sol_mem;
  d_ocp_qp_sol qp_sol;

  void* ipm_arg_mem;
  d_ocp_qp_ipm_arg arg;

  // workspace
  void* ipm_mem;
  d_ocp_qp_ipm_ws workspace;

  int hpipm_status;  // status code after solving

  // todo make this a setting
  ::hpipm_mode mode = ::hpipm_mode::SPEED;  // ROBUST/BALANCED; see also hpipm_common.h
};

HpipmInterface::HpipmInterface(OcpSize ocpSize, const Settings& settings)
    : pImpl_(new HpipmInterface::Impl(std::move(ocpSize), settings)) {}

HpipmInterface::~HpipmInterface() = default;

void HpipmInterface::solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                           std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& stateTrajectory,
                           std::vector<vector_t>& inputTrajectory, bool verbose) {
  pImpl_->solve(x0, dynamics, cost, stateTrajectory, inputTrajectory, verbose);
}

}  // namespace ocs2