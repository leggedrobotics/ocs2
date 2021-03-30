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

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>

#include "ocs2_sqp/ConstraintProjection.h"

namespace ocs2 {
namespace multiple_shooting {

scalar_array_t timeDiscretizationWithEvents(scalar_t initTime, scalar_t finalTime, scalar_t dt, const scalar_array_t& eventTimes,
                                            scalar_t eventDelta) {
  assert(dt > 0);
  assert(finalTime > initTime);
  scalar_array_t timeDiscretization;

  // Initialize
  timeDiscretization.push_back(initTime);
  scalar_t nextEventIdx = lookup::findIndexInTimeArray(eventTimes, initTime);

  // Fill iteratively
  scalar_t nextTime = timeDiscretization.back();
  while (timeDiscretization.back() < finalTime) {
    nextTime = nextTime + dt;
    bool nextTimeIsEvent = false;

    // Check if an event has passed
    if (nextEventIdx < eventTimes.size() && nextTime >= eventTimes[nextEventIdx]) {
      nextTime = eventTimes[nextEventIdx];
      nextTimeIsEvent = true;
      nextEventIdx++;
    }

    // Check if final time has passed
    if (nextTime >= finalTime) {
      nextTime = finalTime;
      nextTimeIsEvent = false;
    }

    const scalar_t newTimePoint = nextTimeIsEvent ? nextTime + eventDelta : nextTime;
    if (newTimePoint > timeDiscretization.back() + 0.5 * dt) {  // Minimum spacing hardcoded here to 0.5*dt, TODO: make parameter.
      timeDiscretization.push_back(newTimePoint);
    } else {  // Points are close together -> overwrite the old point
      timeDiscretization.back() = newTimePoint;
    }
  }

  return timeDiscretization;
}

Transcription setupIntermediateNode(SystemDynamicsBase& systemDynamics, DynamicsSensitivityDiscretizer& sensitivityDiscretizer,
                                    CostFunctionBase& costFunction, ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
                                    bool projectStateInputEqualityConstraints, scalar_t t, scalar_t dt, const vector_t& x,
                                    const vector_t& x_next, const vector_t& u) {
  // Results and short-hand notation
  Transcription transcription;
  auto& dynamics = transcription.dynamics;
  auto& performance = transcription.performance;
  auto& cost = transcription.cost;
  auto& constraints = transcription.constraints;

  // Dynamics
  // Discretization returns // x_{k+1} = A_{k} * dx_{k} + B_{k} * du_{k} + b_{k}
  dynamics = sensitivityDiscretizer(systemDynamics, t, x, u, dt);
  dynamics.f -= x_next;  // make it dx_{k+1} = ...
  performance.stateEqConstraintISE += dt * dynamics.f.squaredNorm();

  // Costs: Approximate the integral with forward euler (correct for dt after adding penalty)
  cost = costFunction.costQuadraticApproximation(t, x, u);
  performance.totalCost += dt * cost.f;

  // Constraints
  if (constraintPtr != nullptr) {
    // Inequalities as penalty
    if (penaltyPtr != nullptr) {
      const auto ineqConstraints = constraintPtr->inequalityConstraintQuadraticApproximation(t, x, u);
      if (ineqConstraints.f.rows() > 0) {
        const auto penaltyCost = penaltyPtr->penaltyCostQuadraticApproximation(ineqConstraints);
        cost += penaltyCost;  // add to cost before potential projection.
        performance.inequalityConstraintISE += dt * ineqConstraints.f.cwiseMin(0.0).squaredNorm();
        performance.inequalityConstraintPenalty += dt * penaltyCost.f;
      }
    }

    // C_{k} * dx_{k} + D_{k} * du_{k} + e_{k} = 0
    constraints = constraintPtr->stateInputEqualityConstraintLinearApproximation(t, x, u);
    performance.stateInputEqConstraintISE += dt * constraints.f.squaredNorm();
    if (projectStateInputEqualityConstraints) {  // Handle equality constraints using projection.
      // Projection stored instead of constraint, // TODO: benchmark between lu and qr method. LU seems slightly faster.
      constraints = luConstraintProjection(constraints);

      // Adapt dynamics and cost
      changeOfInputVariables(dynamics, constraints.dfdu, constraints.dfdx, constraints.f);
      changeOfInputVariables(cost, constraints.dfdu, constraints.dfdx, constraints.f);
    }
  }

  // Costs: Approximate the integral with forward euler  (correct for dt HERE, after adding penalty)
  cost.dfdxx *= dt;
  cost.dfdux *= dt;
  cost.dfduu *= dt;
  cost.dfdx *= dt;
  cost.dfdu *= dt;
  cost.f *= dt;

  return transcription;
}

PerformanceIndex computeIntermediatePerformance(SystemDynamicsBase& systemDynamics, DynamicsDiscretizer& discretizer,
                                                CostFunctionBase& costFunction, ConstraintBase* constraintPtr, PenaltyBase* penaltyPtr,
                                                scalar_t t, scalar_t dt, const vector_t& x, const vector_t& x_next, const vector_t& u) {
  PerformanceIndex performance;

  // Dynamics
  const vector_t dynamicsGap = discretizer(systemDynamics, t, x, u, dt) - x_next;
  performance.stateEqConstraintISE += dt * dynamicsGap.squaredNorm();

  // Costs
  performance.totalCost += dt * costFunction.cost(t, x, u);

  // Constraints
  if (constraintPtr != nullptr) {
    const vector_t constraints = constraintPtr->stateInputEqualityConstraint(t, x, u);
    performance.stateInputEqConstraintISE += dt * constraints.squaredNorm();

    // Inequalities as penalty
    if (penaltyPtr) {
      const vector_t ineqConstraints = constraintPtr->inequalityConstraint(t, x, u);
      if (ineqConstraints.rows() > 0) {
        const scalar_t penaltyCost = penaltyPtr->penaltyCost(ineqConstraints);
        performance.inequalityConstraintISE += dt * ineqConstraints.cwiseMin(0.0).squaredNorm();
        performance.inequalityConstraintPenalty += dt * penaltyCost;
      }
    }
  }

  return performance;
}

TerminalTranscription setupTerminalNode(CostFunctionBase* terminalCostFunctionPtr, ConstraintBase* constraintPtr, scalar_t t,
                                        const vector_t& x) {
  // Results and short-hand notation
  TerminalTranscription transcription;
  auto& performance = transcription.performance;
  auto& cost = transcription.cost;
  auto& constraints = transcription.constraints;

  // Terminal conditions
  if (terminalCostFunctionPtr != nullptr) {
    cost = terminalCostFunctionPtr->finalCostQuadraticApproximation(t, x);
    performance.totalCost += cost.f;
  } else {
    cost = ScalarFunctionQuadraticApproximation::Zero(x.size(), 0);
  }

  constraints = VectorFunctionLinearApproximation::Zero(0, x.size(), 0);
  return transcription;
}

PerformanceIndex computeTerminalPerformance(CostFunctionBase* terminalCostFunctionPtr, ConstraintBase* constraintPtr, scalar_t t,
                                            const vector_t& x) {
  PerformanceIndex performance;
  performance.totalCost += terminalCostFunctionPtr->finalCost(t, x);
  return performance;
}

}  // namespace multiple_shooting
}  // namespace ocs2