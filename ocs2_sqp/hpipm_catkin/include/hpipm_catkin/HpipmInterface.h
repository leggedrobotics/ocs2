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

  /**
   * Return the Riccati cost-to-go for the previously solved problem.
   * Extra information about the initial stage is needed to complete calculation.
   *
   * Cost-to-go at a node is: V_k(x) = 0.5 * x' * dfdxx * x + x' * dfdx + f
   * For the moment, the value for f is set to 0.0 because it is expensive to compute and often not needed.
   *
   * @param dynamics0 : dynamics at k = 0
   * @param cost0 : cost at k = 0
   * @return Sequence of quadratic cost-to-go's.
   */
  std::vector<ScalarFunctionQuadraticApproximation> getRiccatiCostToGo(const VectorFunctionLinearApproximation& dynamics0,
                                                                       const ScalarFunctionQuadraticApproximation& cost0);

  /**
   * Return the sequence of N feedback matrices for the previously solved problem.
   * Extra information about the initial stage is needed to complete calculation.
   *
   * @param dynamics0 : dynamics at k = 0
   * @param cost0 : cost at k = 0
   * @return Sequence of feedback matrices K of the optimal solution u = K x + k
   */
  matrix_array_t getRiccatiFeedback(const VectorFunctionLinearApproximation& dynamics0, const ScalarFunctionQuadraticApproximation& cost0);

  /**
   * Return the sequence of N feedforward input vectors for the previously solved problem.
   * Extra information about the initial stage is needed to complete calculation.
   *
   * @param dynamics0 : dynamics at k = 0
   * @param cost0 : cost at k = 0
   * @return Sequence of feedforward vectors k of the optimal solution u = K x + k
   */
  vector_array_t getRiccatiFeedforward(const VectorFunctionLinearApproximation& dynamics0,
                                       const ScalarFunctionQuadraticApproximation& cost0);

 private:
  class Impl;
  std::unique_ptr<Impl> pImpl_;
};

}  // namespace ocs2