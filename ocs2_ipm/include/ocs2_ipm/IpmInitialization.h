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
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {
namespace ipm {

/**
 * Initializes the slack variable. The initialization of the slack variables follows IPOPT
 * (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Initialization).
 *
 * @param ineqConstraint: The value of the inequality constraint.
 * @param initialSlackLowerBound : Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param initialSlackMarginRate : Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable.
 */
inline vector_t initializeSlackVariable(const vector_t& ineqConstraint, scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ineqConstraint.size() > 0) {
    return (1.0 + initialSlackMarginRate) * ineqConstraint.cwiseMax(initialSlackLowerBound);
  } else {
    return vector_t();
  }
}

/**
 * Initializes the dual variable. The initialization of dual variables follows IPOPT option `bound_mult_init_method`: `mu-based`
 * (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_Initialization) with additoinal options.
 *
 * @param slack: The slack variable.
 * @param barrierParam: The barrier parameter of the IPM. Must be positive.
 * @param initialDualLowerBound : Lower bound of the initial dual variables.
 * @param initialDualMarginRate : Additional margin rate of the initial dual variables.
 * @return Initialized dual variable.
 */
inline vector_t initializeDualVariable(const vector_t& slack, scalar_t barrierParam, scalar_t initialDualLowerBound,
                                       scalar_t initialDualMarginRate) {
  if (slack.size() > 0) {
    return (1.0 + initialDualMarginRate) * (barrierParam * slack.cwiseInverse()).cwiseMax(initialDualLowerBound);
  } else {
    return vector_t();
  }
}

/**
 * Initializes the slack variable at a single intermediate node.
 *
 * @param ocpDefinition: Definition of the optimal control problem.
 * @param time : Time of this node
 * @param state : State
 * @param input : Input
 * @param initialSlackLowerBound : Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param initialSlackMarginRate : Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variables of the intermediate state-only (first) and state-input (second) constraints.
 */
std::pair<vector_t, vector_t> initializeIntermediateSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time,
                                                                  const vector_t& state, const vector_t& input,
                                                                  scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

/**
 * Initializes the slack variable at the terminal node.
 *
 * @param ocpDefinition: Definition of the optimal control problem.
 * @param time : Time at the terminal node
 * @param state : Terminal state
 * @param initialSlackLowerBound : Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param initialSlackMarginRate : Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable of the terminal state-only constraints.
 */
vector_t initializeTerminalSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                         scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

/**
 * Initializes the slack variable at an event node.
 *
 * @param ocpDefinition: Definition of the optimal control problem.
 * @param time : Time at the event node
 * @param state : Pre-event state
 * @param initialSlackLowerBound : Lower bound of the initial slack variables. Corresponds to `slack_bound_push` option of IPOPT.
 * @param initialSlackMarginRate : Additional margin rate of the initial slack variables. Corresponds to `slack_bound_frac` option of IPOPT.
 * @return Initialized slack variable of the pre-jump state-only constraints.
 */
vector_t initializeEventSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                      scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate);

}  // namespace ipm
}  // namespace ocs2