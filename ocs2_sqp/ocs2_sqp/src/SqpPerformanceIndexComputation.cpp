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

#include "ocs2_sqp/SqpPerformanceIndexComputation.h"

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {
namespace sqp {

namespace {
scalar_t getEqConstraintsSSE(const vector_t& eqConstraint) {
  if (eqConstraint.size() == 0) {
    return 0.0;
  } else {
    return eqConstraint.squaredNorm();
  }
}

scalar_t getIneqConstraintsSSE(const vector_t& ineqConstraint) {
  if (ineqConstraint.size() == 0) {
    return 0.0;
  } else {
    return ineqConstraint.cwiseMin(0.0).squaredNorm();
  }
}
}  // namespace

PerformanceIndex computeIntermediatePerformance(const multiple_shooting::Transcription& transcription, scalar_t dt) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = dt * transcription.dynamics.f.squaredNorm();

  // Costs
  performance.cost = transcription.cost.f;

  // State-input equality constraints
  performance.equalityConstraintsSSE = dt * getEqConstraintsSSE(transcription.stateInputEqConstraints.f);

  // Inequality constraints.
  performance.inequalityConstraintsSSE =
      dt * getIneqConstraintsSSE(transcription.stateIneqConstraints.f) + dt * getIneqConstraintsSSE(transcription.stateIneqConstraints.f);

  return performance;
}

PerformanceIndex computeIntermediatePerformance(const OptimalControlProblem& optimalControlProblem, DynamicsDiscretizer& discretizer,
                                                scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  PerformanceIndex performance;

  // Dynamics
  vector_t dynamicsGap = discretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamicsGap -= x_next;
  performance.dynamicsViolationSSE = dt * dynamicsGap.squaredNorm();

  // Precomputation for other terms
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint;
  optimalControlProblem.preComputationPtr->request(request, t, x, u);

  // Costs
  performance.cost = dt * computeCost(optimalControlProblem, t, x, u);

  // State-input equality constraints
  if (!optimalControlProblem.equalityConstraintPtr->empty()) {
    const vector_t stateInputEqConstraints =
        optimalControlProblem.equalityConstraintPtr->getValue(t, x, u, *optimalControlProblem.preComputationPtr);
    performance.equalityConstraintsSSE = dt * getEqConstraintsSSE(stateInputEqConstraints);
  }

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    const vector_t stateIneqConstraints =
        optimalControlProblem.stateInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE = dt * getIneqConstraintsSSE(stateIneqConstraints);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    const vector_t stateInputIneqConstraints =
        optimalControlProblem.inequalityConstraintPtr->getValue(t, x, u, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += dt * getIneqConstraintsSSE(stateInputIneqConstraints);
  }

  return performance;
}

PerformanceIndex computeTerminalPerformance(const multiple_shooting::TerminalTranscription& transcription) {
  PerformanceIndex performance;

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = getEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = getIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex computeTerminalPerformance(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  PerformanceIndex performance;

  constexpr auto request = Request::Cost + Request::SoftConstraint;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  performance.cost = computeFinalCost(optimalControlProblem, t, x);

  if (!optimalControlProblem.finalEqualityConstraintPtr->empty()) {
    const vector_t eqConstraints =
        optimalControlProblem.finalEqualityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.equalityConstraintsSSE = getEqConstraintsSSE(eqConstraints);
  }

  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    const vector_t ineqConstraints =
        optimalControlProblem.finalInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE = getIneqConstraintsSSE(ineqConstraints);
  }

  return performance;
}

PerformanceIndex computeEventPerformance(const multiple_shooting::EventTranscription& transcription) {
  PerformanceIndex performance;

  // Dynamics
  performance.dynamicsViolationSSE = transcription.dynamics.f.squaredNorm();

  performance.cost = transcription.cost.f;

  // Equality constraints
  performance.equalityConstraintsSSE = getEqConstraintsSSE(transcription.eqConstraints.f);

  // State inequality constraints.
  performance.inequalityConstraintsSSE = getIneqConstraintsSSE(transcription.ineqConstraints.f);

  return performance;
}

PerformanceIndex computeEventPerformance(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                         const vector_t& x_next) {
  PerformanceIndex performance;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Dynamics;
  optimalControlProblem.preComputationPtr->requestPreJump(request, t, x);

  // Dynamics
  const vector_t dynamicsGap = optimalControlProblem.dynamicsPtr->computeJumpMap(t, x) - x_next;
  performance.dynamicsViolationSSE = dynamicsGap.squaredNorm();

  performance.cost = computeEventCost(optimalControlProblem, t, x);

  if (!optimalControlProblem.preJumpEqualityConstraintPtr->empty()) {
    const vector_t eqConstraints =
        optimalControlProblem.preJumpEqualityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.equalityConstraintsSSE = getEqConstraintsSSE(eqConstraints);
  }

  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    const vector_t ineqConstraints =
        optimalControlProblem.preJumpInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE = getIneqConstraintsSSE(ineqConstraints);
  }

  return performance;
}

}  // namespace sqp
}  // namespace ocs2
