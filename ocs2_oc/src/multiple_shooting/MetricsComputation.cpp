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

Metrics computeMetrics(const Transcription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Dynamics
  metrics.dynamicsViolation = transcription.dynamics.f;

  // Equality constraints
  metrics.stateEqConstraint = toConstraintArray(constraintsSize.stateEq, transcription.stateEqConstraints.f);
  metrics.stateInputEqConstraint = toConstraintArray(constraintsSize.stateInputEq, transcription.stateInputEqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = toConstraintArray(constraintsSize.stateIneq, transcription.stateIneqConstraints.f);
  metrics.stateInputIneqConstraint = toConstraintArray(constraintsSize.stateInputIneq, transcription.stateInputIneqConstraints.f);

  return metrics;
}

Metrics computeMetrics(const EventTranscription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Dynamics
  metrics.dynamicsViolation = transcription.dynamics.f;

  // Equality constraints
  metrics.stateEqConstraint = toConstraintArray(constraintsSize.stateEq, transcription.eqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = toConstraintArray(constraintsSize.stateIneq, transcription.ineqConstraints.f);

  return metrics;
}

Metrics computeMetrics(const TerminalTranscription& transcription) {
  const auto& constraintsSize = transcription.constraintsSize;

  Metrics metrics;

  // Cost
  metrics.cost = transcription.cost.f;

  // Equality constraints
  metrics.stateEqConstraint = toConstraintArray(constraintsSize.stateEq, transcription.eqConstraints.f);

  // Inequality constraints.
  metrics.stateIneqConstraint = toConstraintArray(constraintsSize.stateIneq, transcription.ineqConstraints.f);

  return metrics;
}

Metrics computeIntermediateMetrics(OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer, scalar_t t, scalar_t dt,
                                   const vector_t& x, const vector_t& x_next, const vector_t& u) {
  // Dynamics
  auto dynamicsViolation = discretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamicsViolation -= x_next;

  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->request(request, t, x, u);

  // Compute metrics
  auto metrics = computeIntermediateMetrics(optimalControlProblem, t, x, u, std::move(dynamicsViolation));
  metrics.cost *= dt;  // consider dt

  return metrics;
}

Metrics computeTerminalMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  return computeFinalMetrics(optimalControlProblem, t, x);
}

Metrics computeEventMetrics(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next) {
  // Precomputation
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Dynamics;
  optimalControlProblem.preComputationPtr->requestPreJump(request, t, x);

  // Dynamics
  auto dynamicsViolation = optimalControlProblem.dynamicsPtr->computeJumpMap(t, x);
  dynamicsViolation -= x_next;

  return computePreJumpMetrics(optimalControlProblem, t, x, std::move(dynamicsViolation));
}

}  // namespace multiple_shooting
}  // namespace ocs2
