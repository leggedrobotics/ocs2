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

#include "ocs2_oc/multiple_shooting/Transcription.h"

#include <ocs2_core/misc/LinearAlgebra.h>

#include "ocs2_oc/approximate_model/ChangeOfInputVariables.h"
#include "ocs2_oc/approximate_model/LinearQuadraticApproximator.h"

namespace ocs2 {
namespace multiple_shooting {

Transcription setupIntermediateNode(OptimalControlProblem& optimalControlProblem, DynamicsSensitivityDiscretizer& sensitivityDiscretizer,
                                    scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  // Results and short-hand notation
  Transcription transcription;
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& constraintsSize = transcription.constraintsSize;
  auto& stateEqConstraints = transcription.stateEqConstraints;
  auto& stateInputEqConstraints = transcription.stateInputEqConstraints;
  auto& stateIneqConstraints = transcription.stateIneqConstraints;
  auto& stateInputIneqConstraints = transcription.stateInputIneqConstraints;

  // Dynamics
  // Discretization returns x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  dynamics = sensitivityDiscretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamics.f -= x_next;  // make it dx_{k+1} = ...

  // Precomputation for other terms
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->request(request, t, x, u);

  // Costs: Approximate the integral with forward euler
  cost = approximateCost(optimalControlProblem, t, x, u);
  cost *= dt;

  // State equality constraints
  if (!optimalControlProblem.stateEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.stateEqualityConstraintPtr->getTermsSize(t);
    stateEqConstraints =
        optimalControlProblem.stateEqualityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input equality constraints
  if (!optimalControlProblem.equalityConstraintPtr->empty()) {
    constraintsSize.stateInputEq = optimalControlProblem.equalityConstraintPtr->getTermsSize(t);
    stateInputEqConstraints =
        optimalControlProblem.equalityConstraintPtr->getLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.stateInequalityConstraintPtr->getTermsSize(t);
    stateIneqConstraints =
        optimalControlProblem.stateInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    constraintsSize.stateInputIneq = optimalControlProblem.inequalityConstraintPtr->getTermsSize(t);
    stateInputIneqConstraints =
        optimalControlProblem.inequalityConstraintPtr->getLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

void projectTranscription(Transcription& transcription, bool extractProjectionMultiplier) {
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& stateInputEqConstraints = transcription.stateInputEqConstraints;
  auto& stateInputIneqConstraints = transcription.stateInputIneqConstraints;
  auto& projection = transcription.constraintsProjection;
  auto& projectionMultiplierCoefficients = transcription.projectionMultiplierCoefficients;

  if (stateInputEqConstraints.f.size() > 0) {
    // Projection stored instead of constraint, // TODO: benchmark between lu and qr method. LU seems slightly faster.
    if (extractProjectionMultiplier) {
      matrix_t constraintPseudoInverse;
      std::tie(projection, constraintPseudoInverse) = LinearAlgebra::qrConstraintProjection(stateInputEqConstraints);
      projectionMultiplierCoefficients.compute(cost, dynamics, projection, constraintPseudoInverse);
    } else {
      projection = LinearAlgebra::luConstraintProjection(stateInputEqConstraints).first;
      projectionMultiplierCoefficients = ProjectionMultiplierCoefficients();
    }
    stateInputEqConstraints = VectorFunctionLinearApproximation();

    // Adapt dynamics, cost, and state-input inequality constraints
    changeOfInputVariables(dynamics, projection.dfdu, projection.dfdx, projection.f);
    changeOfInputVariables(cost, projection.dfdu, projection.dfdx, projection.f);
    if (stateInputIneqConstraints.f.size() > 0) {
      changeOfInputVariables(stateInputIneqConstraints, projection.dfdu, projection.dfdx, projection.f);
    }
  }
}

TerminalTranscription setupTerminalNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  // Results and short-hand notation
  TerminalTranscription transcription;
  auto& cost = transcription.cost;
  auto& constraintsSize = transcription.constraintsSize;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  // Costs
  cost = approximateFinalCost(optimalControlProblem, t, x);

  // State equality constraints.
  if (!optimalControlProblem.finalEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.finalEqualityConstraintPtr->getTermsSize(t);
    eqConstraints =
        optimalControlProblem.finalEqualityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.finalInequalityConstraintPtr->getTermsSize(t);
    ineqConstraints =
        optimalControlProblem.finalInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

EventTranscription setupEventNode(OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x, const vector_t& x_next) {
  // Results and short-hand notation
  EventTranscription transcription;
  auto& cost = transcription.cost;
  auto& dynamics = transcription.dynamics;
  auto& constraintsSize = transcription.constraintsSize;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Dynamics + Request::Approximation;
  optimalControlProblem.preComputationPtr->requestPreJump(request, t, x);

  // Dynamics
  // jump map returns // x_{k+1} = A_{k} * dx_{k} + b_{k}
  dynamics = optimalControlProblem.dynamicsPtr->jumpMapLinearApproximation(t, x);
  dynamics.f -= x_next;                // make it dx_{k+1} = ...
  dynamics.dfdu.setZero(x.size(), 0);  // Overwrite derivative that shouldn't exist.

  // Costs
  cost = approximateEventCost(optimalControlProblem, t, x);

  // State equality constraints.
  if (!optimalControlProblem.preJumpEqualityConstraintPtr->empty()) {
    constraintsSize.stateEq = optimalControlProblem.preJumpEqualityConstraintPtr->getTermsSize(t);
    eqConstraints =
        optimalControlProblem.preJumpEqualityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  // State inequality constraints.
  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    constraintsSize.stateIneq = optimalControlProblem.preJumpInequalityConstraintPtr->getTermsSize(t);
    ineqConstraints =
        optimalControlProblem.preJumpInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
  }

  return transcription;
}

}  // namespace multiple_shooting
}  // namespace ocs2
