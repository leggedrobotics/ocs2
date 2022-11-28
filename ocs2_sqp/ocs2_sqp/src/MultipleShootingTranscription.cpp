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

#include "ocs2_sqp/MultipleShootingTranscription.h"

#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

#include "ocs2_sqp/ConstraintProjection.h"

namespace ocs2 {
namespace multiple_shooting {

namespace {
scalar_t getIneqConstraintsSSE(const vector_t& ineqConstraint) {
  if (ineqConstraint.size() == 0) {
    return 0.0;
  } else {
    return ineqConstraint.cwiseMin(0.0).squaredNorm();
  }
}
}  // namespace

Transcription setupIntermediateNode(const OptimalControlProblem& optimalControlProblem,
                                    DynamicsSensitivityDiscretizer& sensitivityDiscretizer, bool projectStateInputEqualityConstraints,
                                    scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u,
                                    bool extractEqualityConstraintsPseudoInverse) {
  // Results and short-hand notation
  Transcription transcription;
  auto& dynamics = transcription.dynamics;
  auto& performance = transcription.performance;
  auto& cost = transcription.cost;
  auto& projection = transcription.constraintsProjection;
  auto& constraintPseudoInverse = transcription.constraintPseudoInverse;
  auto& stateInputEqConstraints = transcription.stateInputEqConstraints;
  auto& stateIneqConstraints = transcription.stateIneqConstraints;
  auto& stateInputIneqConstraints = transcription.stateInputIneqConstraints;

  // Dynamics
  // Discretization returns x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  dynamics = sensitivityDiscretizer(*optimalControlProblem.dynamicsPtr, t, x, u, dt);
  dynamics.f -= x_next;  // make it dx_{k+1} = ...
  performance.dynamicsViolationSSE = dt * dynamics.f.squaredNorm();

  // Precomputation for other terms
  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Constraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->request(request, t, x, u);

  // Costs: Approximate the integral with forward euler
  cost = approximateCost(optimalControlProblem, t, x, u);
  cost *= dt;
  performance.cost = cost.f;

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    stateIneqConstraints =
        optimalControlProblem.stateInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += dt * getIneqConstraintsSSE(stateIneqConstraints.f);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    stateInputIneqConstraints =
        optimalControlProblem.inequalityConstraintPtr->getLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += dt * getIneqConstraintsSSE(stateInputIneqConstraints.f);
  }

  // State-input equality constraints
  if (!optimalControlProblem.equalityConstraintPtr->empty()) {
    // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
    stateInputEqConstraints =
        optimalControlProblem.equalityConstraintPtr->getLinearApproximation(t, x, u, *optimalControlProblem.preComputationPtr);
    if (stateInputEqConstraints.f.size() > 0) {
      performance.equalityConstraintsSSE = dt * stateInputEqConstraints.f.squaredNorm();
      if (projectStateInputEqualityConstraints) {  // Handle equality constraints using projection.
        // Projection stored instead of constraint, // TODO: benchmark between lu and qr method. LU seems slightly faster.
        if (extractEqualityConstraintsPseudoInverse) {
          std::tie(projection, constraintPseudoInverse) = qrConstraintProjection(stateInputEqConstraints);
        } else {
          projection = luConstraintProjection(stateInputEqConstraints).first;
          constraintPseudoInverse = matrix_t();
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
  }

  return transcription;
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
    performance.equalityConstraintsSSE = dt * stateInputEqConstraints.squaredNorm();
  }

  // State inequality constraints.
  if (!optimalControlProblem.stateInequalityConstraintPtr->empty()) {
    const vector_t stateIneqConstraints =
        optimalControlProblem.stateInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += dt * getIneqConstraintsSSE(stateIneqConstraints);
  }

  // State-input inequality constraints.
  if (!optimalControlProblem.inequalityConstraintPtr->empty()) {
    const vector_t stateInputIneqConstraints =
        optimalControlProblem.inequalityConstraintPtr->getValue(t, x, u, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += dt * getIneqConstraintsSSE(stateInputIneqConstraints);
  }

  return performance;
}

TerminalTranscription setupTerminalNode(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  // Results and short-hand notation
  TerminalTranscription transcription;
  auto& performance = transcription.performance;
  auto& cost = transcription.cost;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Approximation;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  cost = approximateFinalCost(optimalControlProblem, t, x);
  performance.cost = cost.f;

  // State inequality constraints.
  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    ineqConstraints =
        optimalControlProblem.finalInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += getIneqConstraintsSSE(ineqConstraints.f);
  }

  eqConstraints = VectorFunctionLinearApproximation::Zero(0, x.size());

  return transcription;
}

PerformanceIndex computeTerminalPerformance(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x) {
  PerformanceIndex performance;

  constexpr auto request = Request::Cost + Request::SoftConstraint;
  optimalControlProblem.preComputationPtr->requestFinal(request, t, x);

  performance.cost = computeFinalCost(optimalControlProblem, t, x);

  if (!optimalControlProblem.finalInequalityConstraintPtr->empty()) {
    const vector_t ineqConstraints =
        optimalControlProblem.finalInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += getIneqConstraintsSSE(ineqConstraints);
  }

  return performance;
}

EventTranscription setupEventNode(const OptimalControlProblem& optimalControlProblem, scalar_t t, const vector_t& x,
                                  const vector_t& x_next) {
  // Results and short-hand notation
  EventTranscription transcription;
  auto& performance = transcription.performance;
  auto& dynamics = transcription.dynamics;
  auto& cost = transcription.cost;
  auto& eqConstraints = transcription.eqConstraints;
  auto& ineqConstraints = transcription.ineqConstraints;

  constexpr auto request = Request::Cost + Request::SoftConstraint + Request::Dynamics + Request::Approximation;
  optimalControlProblem.preComputationPtr->requestPreJump(request, t, x);

  // Dynamics
  // jump map returns // x_{k+1} = A_{k} * dx_{k} + b_{k}
  dynamics = optimalControlProblem.dynamicsPtr->jumpMapLinearApproximation(t, x);
  dynamics.f -= x_next;                // make it dx_{k+1} = ...
  dynamics.dfdu.setZero(x.size(), 0);  // Overwrite derivative that shouldn't exist.
  performance.dynamicsViolationSSE = dynamics.f.squaredNorm();

  cost = approximateEventCost(optimalControlProblem, t, x);
  performance.cost = cost.f;

  // State inequality constraints.
  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    ineqConstraints =
        optimalControlProblem.preJumpInequalityConstraintPtr->getLinearApproximation(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += getIneqConstraintsSSE(ineqConstraints.f);
  }

  eqConstraints = VectorFunctionLinearApproximation::Zero(0, x.size());
  return transcription;
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

  if (!optimalControlProblem.preJumpInequalityConstraintPtr->empty()) {
    const vector_t ineqConstraints =
        optimalControlProblem.preJumpInequalityConstraintPtr->getValue(t, x, *optimalControlProblem.preComputationPtr);
    performance.inequalityConstraintsSSE += getIneqConstraintsSSE(ineqConstraints);
  }

  return performance;
}

}  // namespace multiple_shooting
}  // namespace ocs2
