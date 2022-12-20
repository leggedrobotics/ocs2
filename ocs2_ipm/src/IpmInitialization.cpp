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

#include "ocs2_ipm/IpmInitialization.h"

namespace ocs2 {
namespace ipm {

std::pair<vector_t, vector_t> initializeIntermediateSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time,
                                                                  const vector_t& state, const vector_t& input,
                                                                  scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  vector_t slackStateIneq, slackStateInputIneq;

  if (!ocpDefinition.stateInequalityConstraintPtr->empty() || !ocpDefinition.inequalityConstraintPtr->empty()) {
    constexpr auto request = Request::Constraint;
    ocpDefinition.preComputationPtr->request(request, time, state, input);
  }

  if (!ocpDefinition.stateInequalityConstraintPtr->empty()) {
    const auto ineqConstraint =
        toVector(ocpDefinition.stateInequalityConstraintPtr->getValue(time, state, *ocpDefinition.preComputationPtr));
    slackStateIneq = initializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
  }

  if (!ocpDefinition.inequalityConstraintPtr->empty()) {
    const auto ineqConstraint =
        toVector(ocpDefinition.inequalityConstraintPtr->getValue(time, state, input, *ocpDefinition.preComputationPtr));
    slackStateInputIneq = initializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
  }

  return std::make_pair(std::move(slackStateIneq), std::move(slackStateInputIneq));
}

vector_t initializeTerminalSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                         scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ocpDefinition.finalInequalityConstraintPtr->empty()) {
    return vector_t();
  }

  constexpr auto request = Request::Constraint;
  ocpDefinition.preComputationPtr->requestFinal(request, time, state);
  const auto ineqConstraint = toVector(ocpDefinition.finalInequalityConstraintPtr->getValue(time, state, *ocpDefinition.preComputationPtr));
  return initializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
}

vector_t initializeEventSlackVariable(OptimalControlProblem& ocpDefinition, scalar_t time, const vector_t& state,
                                      scalar_t initialSlackLowerBound, scalar_t initialSlackMarginRate) {
  if (ocpDefinition.preJumpInequalityConstraintPtr->empty()) {
    return vector_t();
  }

  constexpr auto request = Request::Constraint;
  ocpDefinition.preComputationPtr->requestPreJump(request, time, state);
  const auto ineqConstraint =
      toVector(ocpDefinition.preJumpInequalityConstraintPtr->getValue(time, state, *ocpDefinition.preComputationPtr));
  return initializeSlackVariable(ineqConstraint, initialSlackLowerBound, initialSlackMarginRate);
}

}  // namespace ipm
}  // namespace ocs2