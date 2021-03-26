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

extern "C" {
#include <hpipm_common.h>
}

#include <ocs2_core/Types.h>

#include "hpipm_catkin/HpipmInterfaceSettings.h"
#include "hpipm_catkin/OcpSize.h"

namespace ocs2 {

/**
 * This class implements the interface between Linear Quadratic optimal control problems defined in OCS2 and the HPIPM solver.
 * If the problem dimensions change, resize needs to be called to re-initialize HPIPM.
 */
class HpipmInterface {
 public:
  using OcpSize = hpipm_interface::OcpSize;
  using Settings = hpipm_interface::Settings;

  /**
   * Construct the Hpipm interface with given size and settings.
   * Can directly call solve() for a problem with consistent size.
   */
  explicit HpipmInterface(OcpSize ocpSize = OcpSize(), const Settings& settings = Settings());

  /** Destructor */
  ~HpipmInterface();

  /** Resize the problem */
  void resize(OcpSize ocpSize);

  /**
   * Solves a discrete linear quadratic optimal control problem. The interface needs to be resized to a consistent OcpSize before calling
   * this function
   *
   * The problem should be consistently defined in absolute or delta decision variables in x and u.
   *
   * @param x0 : Initial state (deviation).
   * @param dynamics : Linearized approximation of the discrete dynamics.
   * @param cost : Quadratic approximation of the cost.
   * @param constraints : Linearized approximation of constraints, all constraints are mapped to inequality constraints in HPIPM.
   * @param [out] stateTrajectory : Solution state (deviation) trajectory.
   * @param [out] inputTrajectory : Solution input (deviation) trajectory.
   * @param verbose : Prints the HPIPM iteration statistics if true.
   * @return HPIPM returned with flag hpipm_status::
   *    SUCCESS = QP solved;
   *    MAX_ITER = Maximum number of iterations reached;
   *    MIN_STEP = Minimum step length reached;
   *    NAN_SOL = NaN in computations;
   *    INCONS_EQ = Unconsistent equality constraints;
   */
  hpipm_status solve(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                     std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<VectorFunctionLinearApproximation>* constraints,
                     vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool verbose = false);

  void getRiccatiCostToGo(std::vector<matrix_t>& PMatrics, std::vector<vector_t>& pVectors);
  void getRiccatiFeedbackFeedforward(std::vector<matrix_t>& KMatrics, std::vector<vector_t>& kVectors);
  void getRiccatiZeroStage(const matrix_t& A0, const matrix_t& B0, const vector_t& b0, const matrix_t& Q0, const matrix_t& R0,
                           const matrix_t& S0, const vector_t& q0, const vector_t& r0, matrix_t& P0, matrix_t& K0, vector_t& p0,
                           vector_t& k0);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ocs2