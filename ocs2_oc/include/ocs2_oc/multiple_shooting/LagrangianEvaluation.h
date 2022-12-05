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
 * @param[in] lagrangian : Quadratic approximation of the Lagrangian for a single node.
 * @return SSE of the residual in the dual feasibilities
 */
inline scalar_t evaluateDualFeasibilities(const ScalarFunctionQuadraticApproximation& lagrangian) {
  return lagrangian.dfdx.squaredNorm() + lagrangian.dfdu.squaredNorm();
}

/**
 * Evaluates the quadratic approximation of the Lagrangian for a single intermediate node.
 *
 * @param[in] lmd : Costate at start of the interval
 * @param[in] lmd_next : Costate at the end of the interval
 * @param[in] nu : Lagrange multiplier of the projection constraint.
 * @param[in] dynamics : Linear approximation of the dynamics
 * @param[in] stateInputEqConstraints : Linear approximation of the state-input equality constraints
 * @param[in, out] lagrangian : Should be initially filled with the quadratic approximation of the cost. Then this is overwritten into the
 * quadratic approximation of the Lagrangian.
 */
void evaluateLagrangianIntermediateNode(const vector_t& lmd, const vector_t& lmd_next, const vector_t& nu,
                                        const VectorFunctionLinearApproximation& dynamics,
                                        const VectorFunctionLinearApproximation& stateInputEqConstraints,
                                        ScalarFunctionQuadraticApproximation& lagrangian);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the terminal node.
 *
 * @param[in] lmd : Costate at start of the interval
 * @param[in, out] lagrangian : Should be initially filled with the quadratic approximation of the cost. Then this is overwritten into the
 * quadratic approximation of the Lagrangian.
 */
void evaluateLagrangianTerminalNode(const vector_t& lmd, ScalarFunctionQuadraticApproximation& lagrangian);

/**
 * Evaluates the quadratic approximation of the Lagrangian for the event node.
 *
 * @param[in] lmd : Costate at start of the interval
 * @param[in] lmd_next : Costate at the end of the the interval
 * @param[in] dynamics : Dynamics
 * @param[in, out] lagrangian : Should be initially filled with the quadratic approximation of the cost. Then this is overwritten into the
 * quadratic approximation of the Lagrangian.
 */
void evaluateLagrangianEventNode(const vector_t& lmd, const vector_t& lmd_next, const VectorFunctionLinearApproximation& dynamics,
                                 ScalarFunctionQuadraticApproximation& lagrangian);

}  // namespace multiple_shooting
}  // namespace ocs2