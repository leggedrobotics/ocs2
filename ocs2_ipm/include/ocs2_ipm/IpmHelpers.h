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
#include <ocs2_oc/multiple_shooting/Transcription.h>
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {
namespace ipm {

/**
 * Removes the Newton directions of slack and dual variables and the linearized inequality constraints are removed from the linear system
 * equations for the Newton step computation. These terms are considered in the quadratic approximation of the Lagrangian.
 *
 * @param[in] barrierParam : The barrier parameter of the interior point method.
 * @param[in] slack : The slack variable associated with the inequality constraints.
 * @param[in] dual : The dual variable associated with the inequality constraints.
 * @param[in] ineqConstraints : Linear approximation of the inequality constraints.
 * @param[in, out] lagrangian : Quadratic approximation of the Lagrangian.
 */
void condenseIneqConstraints(scalar_t barrierParam, const vector_t& slack, const vector_t& dual,
                             const VectorFunctionLinearApproximation& ineqConstraints, ScalarFunctionQuadraticApproximation& lagrangian);

/**
 * Computes the SSE of the residual in the perturbed complementary slackness.
 *
 * @param[in] barrierParam : The barrier parameter of the interior point method.
 * @param[in] slack : The slack variable associated with the inequality constraints.
 * @param[in] dual : The dual variable associated with the inequality constraints.
 * @return SSE of the residual in the perturbed complementary slackness
 */
inline scalar_t evaluateComplementarySlackness(scalar_t barrierParam, const vector_t& slack, const vector_t& dual) {
  return (slack.array() * dual.array() - barrierParam).matrix().squaredNorm();
}

/**
 * Retrieves the Newton directions of the slack variable associated with state-input inequality constraints.
 *
 * @param[in] stateInputIneqConstraints : State-input inequality constraints
 * @param[in] dx : Newton direction of the state
 * @param[in] du : Newton direction of the input
 * @param[in] barrierParam : The barrier parameter of the interior point method.
 * @param[in] slackStateInputIneq : The slack variable associated with the state-input inequality constraints.
 * @return Newton directions of the slack variable.
 */
vector_t retrieveSlackDirection(const VectorFunctionLinearApproximation& stateInputIneqConstraints, const vector_t& dx, const vector_t& du,
                                scalar_t barrierParam, const vector_t& slackStateInputIneq);

/**
 * Retrieves the Newton directions of the slack variable associated with state-only inequality constraints.
 *
 * @param[in] stateIneqConstraints : State-only inequality constraints
 * @param[in] dx : Newton direction of the state
 * @param[in] barrierParam : The barrier parameter of the interior point method.
 * @param[in] slackStateIneq : The slack variable associated with the state-only inequality constraints.
 * @return Newton directions of the slack variable.
 */
vector_t retrieveSlackDirection(const VectorFunctionLinearApproximation& stateIneqConstraints, const vector_t& dx, scalar_t barrierParam,
                                const vector_t& slackStateIneq);

/**
 * Retrieves the Newton directions of the dual variable.
 *
 * @param[in] barrierParam : The barrier parameter of the interior point method.
 * @param[in] slack : The slack variable associated with the inequality constraints.
 * @param[in] dual : The dual variable associated with the inequality constraints.
 * @param[in] slackDirection : The Newton direction of the slack variable.
 * @return Newton directions of the dual variable.
 */
vector_t retrieveDualDirection(scalar_t barrierParam, const vector_t& slack, const vector_t& dual, const vector_t& slackDirection);

/**
 * Computes the step size via fraction-to-boundary-rule, which is introduced in the IPOPT's implementaion paper,
 * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
 * https://link.springer.com/article/10.1007/s10107-004-0559-y
 *
 * @param [in] v: A variable (slack or dual).
 * @param [in] dv: The serach direction of the variable (slack dirction or dual dirction).
 * @param [in] marginRate: Margin rate to avoid the slack or dual variables becoming too close to 0. Typical values are 0.95 or 0.995.
 */
scalar_t fractionToBoundaryStepSize(const vector_t& v, const vector_t& dv, scalar_t marginRate = 0.995);

/**
 * Convert the optimized slack or dual trajectories as a DualSolution.
 *
 * @param time: The time discretization.
 * @param constraintsSize: The constraint tems size.
 * @param stateIneq: The slack/dual variable trajectory of the state inequality constraints.
 * @param stateInputIneq: The slack/dual variable trajectory of the state-input inequality constraints.
 * @return A dual solution.
 */
DualSolution toDualSolution(const std::vector<AnnotatedTime>& time, const std::vector<multiple_shooting::ConstraintsSize>& constraintsSize,
                            const vector_array_t& stateIneq, const vector_array_t& stateInputIneq);

/**
 * Extracts slack/dual variables of the state-only and state-input constraints from a MultiplierCollection
 *
 * @param multiplierCollection: The MultiplierCollection.
 * @return slack/dual variables of the state-only (first) and state-input constraints (second).
 */
std::pair<vector_t, vector_t> fromMultiplierCollection(const MultiplierCollection& multiplierCollection);

}  // namespace ipm
}  // namespace ocs2