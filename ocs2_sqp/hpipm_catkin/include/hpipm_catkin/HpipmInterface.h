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

#pragma once

#include <memory>
#include <vector>

extern "C" {
#include <hpipm_common.h>
}

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * This class implements the interface between Linear Quadratic optimal control problems defined in OCS2 and the HPIPM solver.
 * If the problem dimensions change, resize needs to be called to re-initialize HPIPM.
 */
class HpipmInterface {
 public:
  struct OcpSize {
    /// !Need to adapt isSizeEqual in implementation if this struct changes!
    int N;                  // Number of stages
    std::vector<int> nu;    // Number of inputs
    std::vector<int> nx;    // Number of states
    std::vector<int> nbu;   // Number of input box inequality constraints
    std::vector<int> nbx;   // Number of state box inequality constraints
    std::vector<int> ng;    // Number of general inequality constraints
    std::vector<int> nsbu;  // Number of slack variables for input box inequalities
    std::vector<int> nsbx;  // Number of slack variables for state box inequalities
    std::vector<int> nsg;   // Number of slack variables for general inequalities
    OcpSize(int N_, int nx_, int nu_)
        : N(N_),
          nx(N_ + 1, nx_),
          nu(N_ + 1, nu_),
          nbu(N_ + 1, 0),
          nbx(N_ + 1, 0),
          ng(N_ + 1, 0),
          nsbu(N_ + 1, 0),
          nsbx(N_ + 1, 0),
          nsg(N_ + 1, 0) {
      nu.back() = 0;
    }
    OcpSize() : OcpSize(0, 0, 0){};
  };

  struct Settings {
    /// !Need to adapt isSettingsEqual in implementation if this struct changes!
    hpipm_mode hpipmMode = hpipm_mode::SPEED;
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
    Settings(){};
  };

  /**
   * Construct the Hpipm interface with the minimal size and default settings.
   * Will need to call resize() with the correct problem dimensions before calling solve()
   */
  HpipmInterface() : HpipmInterface(OcpSize{0, 0, 0}){};

  /**
   * Construct the Hpipm interface with given size and settings.
   * Can directly call solve() for a problem with consistent size.
   */
  explicit HpipmInterface(OcpSize ocpSize, const Settings& settings = Settings());

  /** Destructor */
  ~HpipmInterface();

  /**
   * Resize with new settings
   */
  void resize(OcpSize ocpSize, const Settings& settings);

  /**
   * Resize with old settings
   */
  void resize(OcpSize ocpSize);

  /**
   * Solves a discrete linear quadratic optimal control problem. The interface needs to be resized to a consistent OcpSize before calling
   * this function
   *
   * The problem should be consistently defined in absolute or delta decision variables in x and u.
   *
   * @param x0 : Initial state (deviation).
   * @param dynamics : Linearized approximation of the discrete dynamics.
   * @param cost : Quadratic approximation of the discrete dynamics.
   * @param constraints : Linearized approximation of constraints, all constraints are mapped to inequality constraints in HPIPM.
   * @param [out] stateTrajectory : Solution state (deviation) trajectory.
   * @param [out] inputTrajectory : Solution input (deviation) trajectory.
   * @param verbose : Prints the HPIPM iteration statistics if true.
   * @return HPIPM returned with flag:
   *    0 = QP solved;
   *    1 = Maximum number of iterations reached;
   *    2 = Minimum step length reached;
   *    3 = NaN in computations;
   *    4 = Unconsistent equality constraints;
   */
  hpipm_status solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                     std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                     std::vector<vector_t>& stateTrajectory, std::vector<vector_t>& inputTrajectory, bool verbose = false);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ocs2