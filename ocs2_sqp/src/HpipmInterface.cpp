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
    dim_mem_ = malloc(dim_size);
    d_ocp_qp_dim_create(ocpSize_.N, &dim_, dim_mem_);
    d_ocp_qp_dim_set_all(ocpSize_.nx.data(), ocpSize_.nu.data(), ocpSize_.nbx.data(), ocpSize_.nbu.data(), ocpSize_.ng.data(),
                         ocpSize_.nsbx.data(), ocpSize_.nsbu.data(), ocpSize_.nsg.data(), &dim_);

    const int qp_size = d_ocp_qp_memsize(&dim_);
    qp_mem_ = malloc(qp_size);
    d_ocp_qp_create(&dim_, &qp_, qp_mem_);

    const int qp_sol_size = d_ocp_qp_sol_memsize(&dim_);
    qp_sol_mem_ = malloc(qp_sol_size);
    d_ocp_qp_sol_create(&dim_, &qp_sol_, qp_sol_mem_);

    const int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim_);
    ipm_arg_mem_ = malloc(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim_, &arg_, ipm_arg_mem_);

    applySettings(settings);

    // Setup workspace after applying the settings
    const int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);
    ipm_mem_ = malloc(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);
  }

  ~Impl() {
    // free the hpipm memories
    free(dim_mem_);
    free(qp_mem_);
    free(qp_sol_mem_);
    free(ipm_arg_mem_);
    free(ipm_mem_);
    dim_mem_ = nullptr;
    qp_mem_ = nullptr;
    qp_sol_mem_ = nullptr;
    ipm_arg_mem_ = nullptr;
    ipm_mem_ = nullptr;
  }

  void applySettings(Settings& settings) {
    // TODO: make this a setting
    ::hpipm_mode mode = ::hpipm_mode::SPEED;  // ROBUST/BALANCED; see also hpipm_common.h
    d_ocp_qp_ipm_arg_set_default(mode, &arg_);

    d_ocp_qp_ipm_arg_set_iter_max(&settings.iter_max, &arg_);
    d_ocp_qp_ipm_arg_set_alpha_min(&settings.alpha_min, &arg_);
    d_ocp_qp_ipm_arg_set_mu0(&settings.mu0, &arg_);
    d_ocp_qp_ipm_arg_set_tol_stat(&settings.tol_stat, &arg_);
    d_ocp_qp_ipm_arg_set_tol_eq(&settings.tol_eq, &arg_);
    d_ocp_qp_ipm_arg_set_tol_ineq(&settings.tol_ineq, &arg_);
    d_ocp_qp_ipm_arg_set_tol_comp(&settings.tol_comp, &arg_);
    d_ocp_qp_ipm_arg_set_reg_prim(&settings.reg_prim, &arg_);
    d_ocp_qp_ipm_arg_set_warm_start(&settings.warm_start, &arg_);
    d_ocp_qp_ipm_arg_set_pred_corr(&settings.pred_corr, &arg_);
    d_ocp_qp_ipm_arg_set_ric_alg(&settings.ric_alg, &arg_);
  }

  void solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
             std::vector<ScalarFunctionQuadraticApproximation>& cost,
             std::vector<VectorFunctionLinearApproximation>* constraints,
             std::vector<vector_t>& stateTrajectory,
             std::vector<vector_t>& inputTrajectory, bool verbose) {
    assert(dynamics.size() == ocpSize_.N);
    assert(cost.size() == (ocpSize_.N + 1));
    // TODO: check state input size.

    // Dynamics
    std::vector<scalar_t*> AA(ocpSize_.N);
    std::vector<scalar_t*> BB(ocpSize_.N);
    std::vector<scalar_t*> bb(ocpSize_.N);

    // Costs (all must be N+1) eventhough nu[N] = 0;
    std::vector<scalar_t*> QQ(ocpSize_.N + 1);
    std::vector<scalar_t*> RR(ocpSize_.N + 1);
    std::vector<scalar_t*> SS(ocpSize_.N + 1);
    std::vector<scalar_t*> qq(ocpSize_.N + 1);
    std::vector<scalar_t*> rr(ocpSize_.N + 1);

    // Constraints (all must be N+1) eventhough nu[N] = 0;
    std::vector<ocs2::vector_t> boundData;
    std::vector<scalar_t*> CC(ocpSize_.N + 1);
    std::vector<scalar_t*> DD(ocpSize_.N + 1);
    std::vector<scalar_t*> llg(ocpSize_.N + 1);
    std::vector<scalar_t*> uug(ocpSize_.N + 1);

    // Dynamics k = 0. Absorb initial state into dynamics
    vector_t b0 = dynamics[0].f;
    b0.noalias() += dynamics[0].dfdx * x0;
    AA[0] = dynamics[0].dfdx.data();
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

    if (constraints != nullptr){
      auto& constr = *constraints;
      // for ocs2 --> C*dx + D*du + e = 0
      // for hpipm --> ug >= C*dx + D*du >= lg
      boundData.reserve(ocpSize_.N);

      for (int k = 0; k < ocpSize_.N; k++) {
        CC[k] = constr[k].dfdx.data();
        DD[k] = constr[k].dfdu.data();
        boundData.emplace_back(-constr[0].f);
        if (k == 0) {  // Initial constraint
          boundData[k].noalias() -= constr[0].dfdx * x0;
        }
        llg[k] = boundData[k].data();
        uug[k] = boundData[k].data();
      }
    }

    int** hidxbx = nullptr;
    scalar_t** hlbx = nullptr;
    scalar_t** hubx = nullptr;
    int** hidxbu = nullptr;
    scalar_t** hlbu = nullptr;
    scalar_t** hubu = nullptr;
    scalar_t** hZl = nullptr;
    scalar_t** hZu = nullptr;
    scalar_t** hzl = nullptr;
    scalar_t** hzu = nullptr;
    int** hidxs = nullptr;
    scalar_t** hlls = nullptr;
    scalar_t** hlus = nullptr;
    d_ocp_qp_set_all(AA.data(), BB.data(), bb.data(), QQ.data(), SS.data(), RR.data(), qq.data(), rr.data(), hidxbx, hlbx, hubx, hidxbu,
                     hlbu, hubu, CC.data(), DD.data(), llg.data(), uug.data(), hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp_);

    d_ocp_qp_ipm_solve(&qp_, &qp_sol_, &arg_, &workspace_);

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
      stateTrajectory[k].resize(ocpSize_.nx[k]);
      d_ocp_qp_sol_get_x(k, &qp_sol_, stateTrajectory[k].data());
    }
  }

  void getInputSolution(std::vector<vector_t>& inputTrajectory) {
    inputTrajectory.resize(ocpSize_.N);
    for (int k = 0; k < ocpSize_.N; ++k) {
      inputTrajectory[k].resize(ocpSize_.nu[k]);
      d_ocp_qp_sol_get_u(k, &qp_sol_, inputTrajectory[k].data());
    }
  }

  void printStatus() {
    int hpipm_status;
    d_ocp_qp_ipm_get_status(&workspace_, &hpipm_status);
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
    d_ocp_qp_ipm_get_iter(&workspace_, &iter);
    scalar_t res_stat;
    d_ocp_qp_ipm_get_max_res_stat(&workspace_, &res_stat);
    scalar_t res_eq;
    d_ocp_qp_ipm_get_max_res_eq(&workspace_, &res_eq);
    scalar_t res_ineq;
    d_ocp_qp_ipm_get_max_res_ineq(&workspace_, &res_ineq);
    scalar_t res_comp;
    d_ocp_qp_ipm_get_max_res_comp(&workspace_, &res_comp);
    scalar_t* stat;
    d_ocp_qp_ipm_get_stat(&workspace_, &stat);
    int stat_m;
    d_ocp_qp_ipm_get_stat_m(&workspace_, &stat_m);
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

  void* dim_mem_;
  d_ocp_qp_dim dim_;

  void* qp_mem_;
  d_ocp_qp qp_;

  void* qp_sol_mem_;
  d_ocp_qp_sol qp_sol_;

  void* ipm_arg_mem_;
  d_ocp_qp_ipm_arg arg_;

  void* ipm_mem_;
  d_ocp_qp_ipm_ws workspace_;
};

HpipmInterface::HpipmInterface(OcpSize ocpSize, const Settings& settings)
    : pImpl_(new HpipmInterface::Impl(std::move(ocpSize), settings)) {}

HpipmInterface::~HpipmInterface() = default;

void HpipmInterface::solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                           std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                           std::vector<vector_t>& stateTrajectory,
                           std::vector<vector_t>& inputTrajectory, bool verbose) {
  pImpl_->solve(x0, dynamics, cost, constraints, stateTrajectory, inputTrajectory, verbose);
}

}  // namespace ocs2