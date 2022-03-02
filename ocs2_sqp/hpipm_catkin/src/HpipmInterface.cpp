/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "hpipm_catkin/HpipmInterface.h"

#include <ocs2_core/misc/LinearAlgebra.h>

extern "C" {
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}

namespace {
/**
 * Manages a block of memory. Allows reuse of memory blocks if the required size does not exceed the old size.
 */
class MemoryBlock {
 public:
  /** Allocate memory of requested size */
  explicit MemoryBlock(size_t size = 0) : ptr_(nullptr), size_(0) { reserve(size); }
  ~MemoryBlock() noexcept { free(ptr_); }

  /**
   * Ensure a block of memory of at least the requested size.
   * Does nothing if the requested size is smaller than equal to the current size.
   * @param size : minimum size of the memory block.
   */
  void reserve(size_t size) {
    if (size > size_) {
      free(ptr_);
      ptr_ = malloc(size);
      if (ptr_ == nullptr) {
        throw std::bad_alloc();
      } else {
        size_ = size;
      }
    }
  }

  /** Get pointer to the memory, might be nullptr */
  void* get() { return ptr_; };

  // Prevents copies
  MemoryBlock(const MemoryBlock&) = delete;
  MemoryBlock& operator=(const MemoryBlock&) = delete;

  // Default move
  MemoryBlock(MemoryBlock&&) = default;
  MemoryBlock& operator=(MemoryBlock&&) = default;

 private:
  void* ptr_;
  size_t size_;
};
}  // namespace

namespace ocs2 {

class HpipmInterface::Impl {
 public:
  Impl(OcpSize ocpSize, Settings settings) : settings_(std::move(settings)) { initializeMemory(std::move(ocpSize), true); }

  void initializeMemory(OcpSize ocpSize, bool forceInitialization = false) {
    // We will remove the initial state from the decision variables before passing the data to HPIPM.
    // This removes the need for adding constraints to enforce x[0] = x_init
    ocpSize.numStates[0] = 0;

    // Skip memory initialization if problem size didn't change.
    if (!forceInitialization && ocpSize_ == ocpSize) {
      return;
    }

    ocpSize_ = std::move(ocpSize);

    const int dim_size = d_ocp_qp_dim_memsize(ocpSize_.numStages);
    dimMem_.reserve(dim_size);
    d_ocp_qp_dim_create(ocpSize_.numStages, &dim_, dimMem_.get());
    d_ocp_qp_dim_set_all(ocpSize_.numStates.data(), ocpSize_.numInputs.data(), ocpSize_.numStateBoxConstraints.data(),
                         ocpSize_.numInputBoxConstraints.data(), ocpSize_.numIneqConstraints.data(), ocpSize_.numStateBoxSlack.data(),
                         ocpSize_.numInputBoxSlack.data(), ocpSize_.numIneqSlack.data(), &dim_);

    const int qp_size = d_ocp_qp_memsize(&dim_);
    qpMem_.reserve(qp_size);
    d_ocp_qp_create(&dim_, &qp_, qpMem_.get());

    const int qp_sol_size = d_ocp_qp_sol_memsize(&dim_);
    qpSolMem_.reserve(qp_sol_size);
    d_ocp_qp_sol_create(&dim_, &qpSol_, qpSolMem_.get());

    const int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim_);
    ipmArgMem_.reserve(ipm_arg_size);
    d_ocp_qp_ipm_arg_create(&dim_, &arg_, ipmArgMem_.get());

    applySettings(settings_);

    // Setup workspace after applying the settings
    const int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim_, &arg_);
    ipmMem_.reserve(ipm_size);
    d_ocp_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipmMem_.get());
  }

  void applySettings(Settings& settings) {
    d_ocp_qp_ipm_arg_set_default(settings.hpipmMode, &arg_);
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

  void verifySizes(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                   std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   std::vector<VectorFunctionLinearApproximation>* constraints) const {
    if (dynamics.size() != ocpSize_.numStages) {
      throw std::runtime_error("[HpipmInterface] Inconsistent size of dynamics: " + std::to_string(dynamics.size()) + " with " +
                               std::to_string(ocpSize_.numStages) + " number of stages.");
    }
    if (cost.size() != ocpSize_.numStages + 1) {
      throw std::runtime_error("[HpipmInterface] Inconsistent size of cost: " + std::to_string(cost.size()) + " with " +
                               std::to_string(ocpSize_.numStages + 1) + " nodes.");
    }
    if (constraints != nullptr) {
      if (constraints->size() != ocpSize_.numStages + 1) {
        throw std::runtime_error("[HpipmInterface] Inconsistent size of constraints: " + std::to_string(constraints->size()) + " with " +
                                 std::to_string(ocpSize_.numStages + 1) + " nodes.");
      }
    }
    // TODO: expand with state-input size checks
  }

  hpipm_status solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                     std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                     vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool verbose) {
    const int N = ocpSize_.numStages;
    verifySizes(x0, dynamics, cost, constraints);

    // === Dynamics ===
    std::vector<scalar_t*> AA(N, nullptr);
    std::vector<scalar_t*> BB(N, nullptr);
    std::vector<scalar_t*> bb(N, nullptr);

    // k = 0. Absorb initial state into dynamics
    // The initial state is removed from the decision variables
    // The first dynamics becomes:
    //    x[1] = A[0]*x[0] + B[0]*u[0] + b[0]
    //         = B[0]*u[0] + (b[0] + A[0]*x[0])
    //         = B[0]*u[0] + \tilde{b}[0]
    // numState[0] = 0 --> No need to specify A[0] here
    vector_t b0 = dynamics[0].f;
    b0.noalias() += dynamics[0].dfdx * x0;
    BB[0] = dynamics[0].dfdu.data();
    bb[0] = b0.data();

    // k = 1 -> N-1
    for (int k = 1; k < N; k++) {
      AA[k] = dynamics[k].dfdx.data();
      BB[k] = dynamics[k].dfdu.data();
      bb[k] = dynamics[k].f.data();
    }

    // === Costs ===
    std::vector<scalar_t*> QQ(N + 1, nullptr);
    std::vector<scalar_t*> RR(N + 1, nullptr);
    std::vector<scalar_t*> SS(N + 1, nullptr);
    std::vector<scalar_t*> qq(N + 1, nullptr);
    std::vector<scalar_t*> rr(N + 1, nullptr);

    // k = 0. Elimination of initial state requires cost adaptation
    // numState[0] = 0 --> No need to specify Q[0], S[0], q[0] here
    vector_t r0 = cost[0].dfdu;
    r0 += cost[0].dfdux * x0;
    RR[0] = cost[0].dfduu.data();
    rr[0] = r0.data();

    // k = 1 -> (N-1)
    for (int k = 1; k < N; k++) {
      QQ[k] = cost[k].dfdxx.data();
      RR[k] = cost[k].dfduu.data();
      SS[k] = cost[k].dfdux.data();
      qq[k] = cost[k].dfdx.data();
      rr[k] = cost[k].dfdu.data();
    }

    // k = N, no inputs
    QQ[N] = cost[N].dfdxx.data();
    qq[N] = cost[N].dfdx.data();

    // === Constraints ===
    // for ocs2 --> C*dx + D*du + e = 0
    // for hpipm --> ug >= C*dx + D*du >= lg
    std::vector<scalar_t*> CC(N + 1, nullptr);
    std::vector<scalar_t*> DD(N + 1, nullptr);
    std::vector<scalar_t*> llg(N + 1, nullptr);
    std::vector<scalar_t*> uug(N + 1, nullptr);
    std::vector<ocs2::vector_t> boundData;  // Declare at this scope to keep the data alive while HPIPM has the pointers

    if (constraints != nullptr) {
      auto& constr = *constraints;
      boundData.resize(N + 1);

      // k = 0, eliminate initial state
      // numState[0] = 0 --> No need to specify C[0] here
      if (constr[0].f.size() > 0) {
        boundData[0] = -constr[0].f;
        boundData[0].noalias() -= constr[0].dfdx * x0;
        llg[0] = boundData[0].data();
        uug[0] = boundData[0].data();
        DD[0] = constr[0].dfdu.data();
      }

      // k = 1 -> (N-1)
      for (int k = 1; k < N; k++) {
        if (constr[k].f.size() > 0) {
          CC[k] = constr[k].dfdx.data();
          DD[k] = constr[k].dfdu.data();
          boundData[k] = -constr[k].f;
          llg[k] = boundData[k].data();
          uug[k] = boundData[k].data();
        }
      }

      // k = N, no inputs
      if (constr[N].f.size() > 0) {
        CC[N] = constr[N].dfdx.data();
        boundData[N] = -constr[N].f;
        llg[N] = boundData[N].data();
        uug[N] = boundData[N].data();
      }
    }

    // === Unused ===
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

    // === Set and solve ===
    d_ocp_qp_set_all(AA.data(), BB.data(), bb.data(), QQ.data(), SS.data(), RR.data(), qq.data(), rr.data(), hidxbx, hlbx, hubx, hidxbu,
                     hlbu, hubu, CC.data(), DD.data(), llg.data(), uug.data(), hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp_);
    d_ocp_qp_ipm_solve(&qp_, &qpSol_, &arg_, &workspace_);

    if (verbose) {
      printStatus();
    }

    if (!getStateSolution(x0, stateTrajectory)) {
      return hpipm_status::NAN_SOL;
    }
    if (!getInputSolution(inputTrajectory)) {
      return hpipm_status::NAN_SOL;
    }

    // Return solver status
    int hpipmStatus = -1;
    d_ocp_qp_ipm_get_status(&workspace_, &hpipmStatus);
    return hpipm_status(hpipmStatus);
  }

  bool getStateSolution(const vector_t& x0, vector_array_t& stateTrajectory) {
    stateTrajectory.resize(ocpSize_.numStages + 1);
    stateTrajectory.front() = x0;
    for (int k = 1; k < (ocpSize_.numStages + 1); ++k) {
      stateTrajectory[k].resize(ocpSize_.numStates[k]);
      d_ocp_qp_sol_get_x(k, &qpSol_, stateTrajectory[k].data());

      if (!stateTrajectory[k].allFinite()) {
        return false;
      }
    }
    return true;
  }

  bool getInputSolution(vector_array_t& inputTrajectory) {
    inputTrajectory.resize(ocpSize_.numStages);
    for (int k = 0; k < ocpSize_.numStages; ++k) {
      inputTrajectory[k].resize(ocpSize_.numInputs[k]);
      d_ocp_qp_sol_get_u(k, &qpSol_, inputTrajectory[k].data());

      if (!inputTrajectory[k].allFinite()) {
        return false;
      }
    }
    return true;
  }

  matrix_array_t getRiccatiFeedback(const VectorFunctionLinearApproximation& dynamics0, const ScalarFunctionQuadraticApproximation& cost0) {
    const int N = ocpSize_.numStages;
    matrix_array_t RiccatiFeedback(N);

    // k = 0, state is not a decision variable. Reconstruct backward pass from k = 1
    matrix_t P1(ocpSize_.numStates[1], ocpSize_.numStates[1]);
    d_ocp_qp_ipm_get_ric_P(&qp_, &arg_, &workspace_, 1, P1.data());

    matrix_t Lr(ocpSize_.numInputs[0], ocpSize_.numInputs[0]);
    d_ocp_qp_ipm_get_ric_Lr(&qp_, &arg_, &workspace_, 0, Lr.data());  // Lr matrix is lower triangular
    LinearAlgebra::setTriangularMinimumEigenvalues(Lr);

    // RiccatiFeedback[0] = - (inv(Lr)^T * inv(Lr)) * (S0 + B0^T * P1 * A0)
    RiccatiFeedback[0] = -cost0.dfdux;
    const matrix_t P1_A0 = P1 * dynamics0.dfdx;
    RiccatiFeedback[0].noalias() -= dynamics0.dfdu.transpose() * P1_A0;
    Lr.triangularView<Eigen::Lower>().solveInPlace(RiccatiFeedback[0]);
    Lr.triangularView<Eigen::Lower>().transpose().solveInPlace(RiccatiFeedback[0]);

    // k > 0
    matrix_t Ls;
    for (int k = 1; k < N; ++k) {
      const auto numInput = ocpSize_.numInputs[k];
      if (numInput > 0) {
        // RiccatiFeedback[k] = -(Ls * Lr.inverse()).transpose();
        Lr.resize(numInput, numInput);
        d_ocp_qp_ipm_get_ric_Lr(&qp_, &arg_, &workspace_, k, Lr.data());  // Lr matrix is lower triangular
        LinearAlgebra::setTriangularMinimumEigenvalues(Lr);

        Ls.resize(ocpSize_.numStates[k], numInput);
        d_ocp_qp_ipm_get_ric_Ls(&qp_, &arg_, &workspace_, k, Ls.data());
        RiccatiFeedback[k].noalias() = -Lr.triangularView<Eigen::Lower>().transpose().solve(Ls.transpose());
      }
    }

    return RiccatiFeedback;
  }

  vector_array_t getRiccatiFeedforward(const VectorFunctionLinearApproximation& dynamics0,
                                       const ScalarFunctionQuadraticApproximation& cost0) {
    const int N = ocpSize_.numStages;
    vector_array_t RiccatiFeedforward(N);

    // k = 0, state is not a decision variable. Reconstruct backward pass from k = 1
    matrix_t P1(ocpSize_.numStates[1], ocpSize_.numStates[1]);
    d_ocp_qp_ipm_get_ric_P(&qp_, &arg_, &workspace_, 1, P1.data());

    matrix_t Lr(ocpSize_.numInputs[0], ocpSize_.numInputs[0]);
    d_ocp_qp_ipm_get_ric_Lr(&qp_, &arg_, &workspace_, 0, Lr.data());
    LinearAlgebra::setTriangularMinimumEigenvalues(Lr);

    vector_t p1(ocpSize_.numStates[1]);
    d_ocp_qp_ipm_get_ric_p(&qp_, &arg_, &workspace_, 1, p1.data());

    // RiccatiFeedforward[0] = -(inv(Lr)^T * inv(Lr)) * (r0 + B0.transpose() * p1 + B0.transpose() * P1 * b0);
    RiccatiFeedforward[0] = -cost0.dfdu;
    p1.noalias() += P1 * dynamics0.f;  // p1 + P1 * b0
    RiccatiFeedforward[0].noalias() -= dynamics0.dfdu.transpose() * p1;
    Lr.triangularView<Eigen::Lower>().solveInPlace(RiccatiFeedforward[0]);
    Lr.triangularView<Eigen::Lower>().transpose().solveInPlace(RiccatiFeedforward[0]);

    // k > 0
    for (int k = 1; k < N; ++k) {
      RiccatiFeedforward[k].resize(ocpSize_.numInputs[k]);
      d_ocp_qp_ipm_get_ric_k(&qp_, &arg_, &workspace_, k, RiccatiFeedforward[k].data());
    }

    return RiccatiFeedforward;
  }

  std::vector<ScalarFunctionQuadraticApproximation> getRiccatiCostToGo(const VectorFunctionLinearApproximation& dynamics0,
                                                                       const ScalarFunctionQuadraticApproximation& cost0) {
    /*
     * Note on notation: HPIPM uses P, p for the cost-to-go, where we use Sm, sv
     */
    const int N = ocpSize_.numStages;
    std::vector<ScalarFunctionQuadraticApproximation> RiccatiCostToGo(N + 1);

    // k > 0, this first so we have P[1] ready for P[0].
    for (int k = 1; k <= N; k++) {
      RiccatiCostToGo[k].dfdxx.resize(ocpSize_.numStates[k], ocpSize_.numStates[k]);
      RiccatiCostToGo[k].dfdx.resize(ocpSize_.numStates[k]);
      d_ocp_qp_ipm_get_ric_P(&qp_, &arg_, &workspace_, k, RiccatiCostToGo[k].dfdxx.data());
      d_ocp_qp_ipm_get_ric_p(&qp_, &arg_, &workspace_, k, RiccatiCostToGo[k].dfdx.data());
    }

    // k = 0
    matrix_t Lr0(ocpSize_.numInputs[0], ocpSize_.numInputs[0]);
    d_ocp_qp_ipm_get_ric_Lr(&qp_, &arg_, &workspace_, 0, Lr0.data());
    LinearAlgebra::setTriangularMinimumEigenvalues(Lr0);

    // Shorthand notation
    const matrix_t& A0 = dynamics0.dfdx;
    const matrix_t& B0 = dynamics0.dfdu;
    const vector_t& b0 = dynamics0.f;
    const matrix_t& Q0 = cost0.dfdxx;
    matrix_t tmp1 = cost0.dfdux;
    const vector_t& q0 = cost0.dfdx;
    vector_t tmp2 = cost0.dfdu;
    const matrix_t& P1 = RiccatiCostToGo[1].dfdxx;
    vector_t tmp3 = RiccatiCostToGo[1].dfdx;

    // Matrix terms
    // RiccatiCostToGo[0].dfdxx = Q0 + A0.transpose() * P1 * A0 -
    //                              (S0 + B0.transpose() * P1 * A0).transpose() * (R0 + B0.transpose() * P1 * B0).inverse() *
    //                                  (S0 + B0.transpose() * P1 * A0)
    // Use that inv(Lr0)^T * inv(Lr0) = (R0 + B0.transpose() * P1 * B0).inverse();
    const matrix_t P1_A0 = P1 * A0;
    tmp1.noalias() += B0.transpose() * P1_A0;
    Lr0.triangularView<Eigen::Lower>().solveInPlace(tmp1);  // tmp1 = inv(Lr0) * (S0.transpose() + A0.transpose() * P1 * B0)
    RiccatiCostToGo[0].dfdxx = Q0;
    RiccatiCostToGo[0].dfdxx.noalias() += A0.transpose() * P1_A0;
    RiccatiCostToGo[0].dfdxx.noalias() -= tmp1.transpose() * tmp1;

    // Vector terms
    // RiccatiCostToGo[0].dfdx = qk + A0.transpose() * p1 + A0.transpose() * P1 * b0 -
    //                   (S0.transpose() + A0.transpose() * P1 * B0) * (R0 + B0.transpose() * P1 * B0).inverse() *
    //                       (r0 + B0.transpose() * p1 + B0.transpose() * P1 * b0);
    tmp3.noalias() += P1 * b0;  // tmp3 = p1 + B0.transpose() * P1 * b0
    tmp2.noalias() += B0.transpose() * tmp3;
    Lr0.triangularView<Eigen::Lower>().solveInPlace(tmp2);  // tmp2 = inv(Lr0) * (r0 + B0.transpose() * p1 + B0.transpose() * P1 * b0)
    RiccatiCostToGo[0].dfdx = q0;
    RiccatiCostToGo[0].dfdx.noalias() += A0.transpose() * tmp3;
    RiccatiCostToGo[0].dfdx.noalias() -= tmp1.transpose() * tmp2;
    return RiccatiCostToGo;
  }

  void printStatus() {
    int hpipmStatus = -1;
    d_ocp_qp_ipm_get_status(&workspace_, &hpipmStatus);
    fprintf(stderr, "\n=== HPIPM ===\n");
    fprintf(stderr, "HPIPM returned with flag %i. -> ", hpipmStatus);
    if (hpipmStatus == hpipm_status::SUCCESS) {
      fprintf(stderr, "QP solved!\n");
    } else if (hpipmStatus == hpipm_status::MAX_ITER) {
      fprintf(stderr, "Solver failed! Maximum number of iterations reached\n");
    } else if (hpipmStatus == hpipm_status::MIN_STEP) {
      fprintf(stderr, "Solver failed! Minimum step length reached\n");
    } else if (hpipmStatus == hpipm_status::NAN_SOL) {
      fprintf(stderr, "Solver failed! NaN in computations\n");
    } else if (hpipmStatus == hpipm_status::INCONS_EQ) {
      fprintf(stderr, "Solver failed! Unconsistent equality constraints\n");
    } else {
      fprintf(stderr, "Solver failed! Unknown return flag\n");
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
    fprintf(stderr, "ipm iter = %d\n", iter);
    fprintf(stderr, "ipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);
    fprintf(stderr,
            "\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref "
            "pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
    {  // print Stats. Implementation adapted from d_print_exp_tran_mat
      for (int j = 0; j < iter + 1; j++) {
        for (int i = 0; i < stat_m; i++) {
          fprintf(stderr, "%e\t", stat[i + stat_m * j]);
        }
        fprintf(stderr, "\n");
      }
    }
  }

 private:
  Settings settings_;
  OcpSize ocpSize_;

  MemoryBlock dimMem_;
  d_ocp_qp_dim dim_;

  MemoryBlock qpMem_;
  d_ocp_qp qp_;

  MemoryBlock qpSolMem_;
  d_ocp_qp_sol qpSol_;

  MemoryBlock ipmArgMem_;
  d_ocp_qp_ipm_arg arg_;

  MemoryBlock ipmMem_;
  d_ocp_qp_ipm_ws workspace_;
};

HpipmInterface::HpipmInterface(OcpSize ocpSize, const Settings& settings)
    : pImpl_(new HpipmInterface::Impl(std::move(ocpSize), settings)) {}

HpipmInterface::~HpipmInterface() = default;

void HpipmInterface::resize(OcpSize ocpSize) {
  pImpl_->initializeMemory(std::move(ocpSize));
}

hpipm_status HpipmInterface::solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                   std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                   std::vector<VectorFunctionLinearApproximation>* constraints, vector_array_t& stateTrajectory,
                                   vector_array_t& inputTrajectory, bool verbose) {
  return pImpl_->solve(x0, dynamics, cost, constraints, stateTrajectory, inputTrajectory, verbose);
}

std::vector<ScalarFunctionQuadraticApproximation> HpipmInterface::getRiccatiCostToGo(const VectorFunctionLinearApproximation& dynamics0,
                                                                                     const ScalarFunctionQuadraticApproximation& cost0) {
  return pImpl_->getRiccatiCostToGo(dynamics0, cost0);
}
matrix_array_t HpipmInterface::getRiccatiFeedback(const VectorFunctionLinearApproximation& dynamics0,
                                                  const ScalarFunctionQuadraticApproximation& cost0) {
  return pImpl_->getRiccatiFeedback(dynamics0, cost0);
}
vector_array_t HpipmInterface::getRiccatiFeedforward(const VectorFunctionLinearApproximation& dynamics0,
                                                     const ScalarFunctionQuadraticApproximation& cost0) {
  return pImpl_->getRiccatiFeedforward(dynamics0, cost0);
}

}  // namespace ocs2
