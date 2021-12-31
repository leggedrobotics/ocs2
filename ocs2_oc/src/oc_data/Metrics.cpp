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

#include "ocs2_oc/oc_data/Metrics.h"

#include <algorithm>
#include <iostream>

#include <ocs2_core/PreComputation.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
IntermediateMetrics computeIntermediateMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x, const vector_t& u) {
  auto& preComputation = *problem.preComputationPtr;
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;

  IntermediateMetrics metrics;

  // cost
  metrics.cost = computeCost(problem, t, x, u);

  // state equality constraint
  //    std::tie(metrics.stateEqConstraint, metrics.stateEqPenalty) =
  //        problem.stateEqualityConstraintPtr->getValue(t, x, l.stateEq, preComputation);
  metrics.stateEqPenalty = problem.stateEqualityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  // state-input equality constraint
  metrics.stateInputEqConstraint = problem.equalityConstraintPtr->getValue(t, x, u, preComputation);

  // state inequality constraint
  //    std::tie(metrics.stateIneqConstraint, metrics.stateIneqPenalty) =
  //        problem.stateInequalityConstraintPtr->getValue(t, x, l.stateIneq, preComputation);
  metrics.stateIneqPenalty = problem.stateInequalityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  // state-input inequality constraints
  //    std::tie(metrics.stateInputIneqConstraint, metrics.stateInputIneqPenalty) =
  //        problem.inequalityConstraintPtr->getValue(t, x, u, l.stateInputIneq, preComputation);
  metrics.stateInputIneqPenalty = problem.inequalityConstraintPtr->getValue(t, x, u, targetTrajectories, preComputation);

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EventMetrics computeEventMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x) {
  auto& preComputation = *problem.preComputationPtr;
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;

  EventMetrics metrics;

  // cost
  metrics.cost = computeEventCost(problem, t, x);

  // state equality constraint
  //     std::tie(metrics.stateEqConstraint, metrics.stateEqPenalty) =
  //         problem.preJumpEqualityConstraintPtr->getValue(t, x, l.stateEq, preComputation);
  metrics.stateEqPenalty = problem.preJumpEqualityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  // state inequality constraint
  //     std::tie(metrics.stateIneqConstraint, metrics.stateIneqPenalty) =
  //         problem.preJumpInequalityConstraintPtr->getValue(t, x, l.stateIneq, preComputation);
  metrics.stateIneqPenalty = problem.preJumpInequalityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  return metrics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EventMetrics computeFinalMetrics(OptimalControlProblem& problem, scalar_t t, const vector_t& x) {
  auto& preComputation = *problem.preComputationPtr;
  const auto& targetTrajectories = *problem.targetTrajectoriesPtr;

  EventMetrics metrics;

  // cost
  metrics.cost = computeFinalCost(problem, t, x);

  // state equality constraint
  //    std::tie(metrics.stateEqConstraint, metrics.stateEqPenalty) =
  //        problem.finalEqualityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);
  metrics.stateEqPenalty = problem.finalEqualityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  // state inequality constraint
  //    std::tie(metrics.stateIneqConstraint, metrics.stateIneqPenalty) =
  //        problem.finalInequalityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);
  metrics.stateIneqPenalty = problem.finalInequalityConstraintPtr->getValue(t, x, targetTrajectories, preComputation);

  return metrics;
}

}  // namespace ocs2
