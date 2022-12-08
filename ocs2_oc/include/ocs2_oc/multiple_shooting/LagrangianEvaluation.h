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

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace multiple_shooting {

/**
 * Computes the SSE of the residual in the dual feasibilities.
 *
 * @param lagrangian : Quadratic approximation of the Lagrangian for a single node.
 * @return SSE of the residual in the dual feasibilities
 */
inline scalar_t evaluateDualFeasibilities(const ScalarFunctionQuadraticApproximation& lagrangian) {
  return lagrangian.dfdx.squaredNorm() + lagrangian.dfdu.squaredNorm();
}

/**
 * Evaluates the quadratic approximation of the Lagrangian for a single intermediate node.
 *
 * @param lmd : Costate at start of the interval
 * @param lmd_next : Costate at the end of the interval
 * @param nu : Lagrange multiplier of the projection constraint.
 * @param cost : Quadratic approximation of the cost.
 * @param dynamics : Linear approximation of the dynamics
 * @param stateInputEqConstraints : Linear approximation of the state-input equality constraints
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation evaluateLagrangianIntermediateNode(const vector_t& lmd, const vector_t& lmd_next, const vector_t& nu,
                                                                        ScalarFunctionQuadraticApproximation&& cost,
                                                                        const VectorFunctionLinearApproximation& dynamics,
                                                                        const VectorFunctionLinearApproximation& stateInputEqConstraints);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the terminal node.
 *
 * @param lmd : Costate at start of the interval
 * @param cost : Quadratic approximation of the cost.
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation evaluateLagrangianTerminalNode(const vector_t& lmd, ScalarFunctionQuadraticApproximation&& cost);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the event node.
 *
 * @param lmd : Costate at start of the interval
 * @param lmd_next : Costate at the end of the the interval
 * @param cost : Quadratic approximation of the cost.
 * @param dynamics : Dynamics
 * @return Quadratic approximation of the Lagrangian.
 */
ScalarFunctionQuadraticApproximation evaluateLagrangianEventNode(const vector_t& lmd, const vector_t& lmd_next,
                                                                 ScalarFunctionQuadraticApproximation&& cost,
                                                                 const VectorFunctionLinearApproximation& dynamics);

}  // namespace multiple_shooting
}  // namespace ocs2