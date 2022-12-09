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

#include "ocs2_oc/multiple_shooting/MetricsComputation.h"

#include "ocs2_oc/approximate_model/LinearQuadraticApproximator.h"

namespace ocs2 {
namespace multiple_shooting {

Metrics computeIntermediateMetrics(const OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t,
                                   scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  Metrics metrics;

  // Dynamics
  metrics.dynamicsViolation = discretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  metrics.dynamicsViolation -= x_next;

  // Precomputation for other terms
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->request(request, t, x, u);

  // Costs
  metrics.cost = dt * computeCost(optimalControlProblem, t, x, u);

  // State equality constraints
  if (!optimalControlProblem.stateEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint = optimalControlProblem.stateEqualityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input equality constraints
  if (!optimalControlProblem.equalityConstraintPtr->empty()) {
    metrics.stateInputEqConstraint =
        optimalControlProblem.equalityConstraintPtr->getValue(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint =
        optimalControlProblem.stateInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    metrics.stateInputIneqConstraint =
        optimalControlProblem.inequalityConstraintPtr->getValue(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  return metrics;
}

Metrics computeTerminalMetrics(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  Metrics metrics;

  constexpr auto request = Request::Cost + Request::SoftConstraint;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  metrics.cost = computeFinalCost(optimalControlProblem, t, x);

  if (!optimalControlProblem.finalEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint = optimalControlProblem.finalEqualityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint =
        optimalControlProblem.finalInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  return metrics;
}

Metrics computeEventMetrics(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next) {
  Metrics metrics;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Dynamics;
  optimalControlProblem.preComputationPtr->requestPreJump(request, t, x);

  // Dynamics
  metrics.dynamicsViolation = optimalControlProblem.dynamicsPtr->computeJumpMap(t, x) - x_next;

  metrics.cost = computeEventCost(optimalControlProblem, t, x);

  if (!optimalControlProblem.preJumpEqualityConstraintPtr->empty()) {
    metrics.stateEqConstraint =
        optimalControlProblem.preJumpEqualityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    metrics.stateIneqConstraint =
        optimalControlProblem.preJumpInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
  }

  return metrics;
}

}  // namespace multiple_shooting
}  // namespace ocs2
