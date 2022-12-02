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

#include "ocs2_oc/search_strategy/FilterLinesearch.h"

namespace ocs2 {

std::pair<bool, FilterLinesearch::StepType> FilterLinesearch::acceptStep(const PerformanceIndex& baselinePerformance,
                                                                         const PerformanceIndex& stepPerformance,
                                                                         scalar_t armijoDescentMetric) const {
  const scalar_t baselineConstraintViolation = totalConstraintViolation(baselinePerformance);
  const scalar_t stepConstraintViolation = totalConstraintViolation(stepPerformance);

  // Step acceptance and record step type
  if (stepConstraintViolation > g_max) {
    // High constraint violation. Only accept decrease in constraints.
    const bool accepted = stepConstraintViolation < ((1.0 - gamma_c) * baselineConstraintViolation);
    return std::make_pair(accepted, StepType::CONSTRAINT);

  } else if (stepConstraintViolation < g_min && baselineConstraintViolation < g_min && armijoDescentMetric < 0.0) {
    // With low violation and having a descent direction, require the armijo condition.
    const bool accepted = stepPerformance.merit < (baselinePerformance.merit + armijoFactor * armijoDescentMetric);
    return std::make_pair(accepted, StepType::COST);

  } else {
    // Medium violation: either merit or constraints decrease (with small gamma_c mixing of old constraints)
    const bool accepted = stepPerformance.merit < (baselinePerformance.merit - gamma_c * baselineConstraintViolation) ||
                          stepConstraintViolation < ((1.0 - gamma_c) * baselineConstraintViolation);
    return std::make_pair(accepted, StepType::DUAL);
  }
}

std::string toString(const FilterLinesearch::StepType& stepType) {
  using StepType = FilterLinesearch::StepType;
  switch (stepType) {
    case StepType::COST:
      return "Cost";
    case StepType::CONSTRAINT:
      return "Constraint";
    case StepType::DUAL:
      return "Dual";
    case StepType::ZERO:
      return "Zero";
    case StepType::UNKNOWN:
    default:
      return "Unknown";
  }
}

scalar_t armijoDescentMetric(const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& deltaXSol,
                             const vector_array_t& deltaUSol) {
  // To determine if the solution is a descent direction for the cost: compute gradient(cost)' * [dx; du]
  scalar_t metric = 0.0;
  for (int i = 0; i < cost.size(); i++) {
    if (cost[i].dfdx.size() > 0) {
      metric += cost[i].dfdx.dot(deltaXSol[i]);
    }
    if (cost[i].dfdu.size() > 0) {
      metric += cost[i].dfdu.dot(deltaUSol[i]);
    }
  }
  return metric;
}

}  // namespace ocs2
