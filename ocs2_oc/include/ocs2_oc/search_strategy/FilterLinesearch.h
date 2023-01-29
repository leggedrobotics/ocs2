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
#include <ocs2_oc/oc_data/PerformanceIndex.h>

namespace ocs2 {

/*
 * Filter linesearch based on:
 * "On the implementation of an interior-point filter line-search algorithm for large-scale nonlinear programming"
 * https://link.springer.com/article/10.1007/s10107-004-0559-y
 *
 * step acceptance criteria with c = costs, g = the norm of constraint violation, and w = [x; u]
 */
struct FilterLinesearch {
  enum class StepType { UNKNOWN, CONSTRAINT, DUAL, COST, ZERO };

  scalar_t g_max = 1e6;          // (1): IF g{i+1} > g_max REQUIRE g{i+1} < (1-gamma_c) * g{i}
  scalar_t g_min = 1e-6;         // (2): ELSE IF (g{i} < g_min AND g{i+1} < g_min AND dc/dw'{i} * delta_w < 0) REQUIRE Armijo condition
  scalar_t gamma_c = 1e-6;       // (3): ELSE REQUIRE c{i+1} < (c{i} - gamma_c * g{i}) OR g{i+1} < (1-gamma_c) * g{i}
  scalar_t armijoFactor = 1e-4;  // Armijo condition: c{i+1} < c{i} + armijoFactor * armijoDescentMetric{i}

  /**
   * Checks that the step is accepted.
   *
   * @param [in] baselinePerformance : The zero step PerformanceIndex
   * @param [in] stepPerformance : The step PerformanceIndex
   * @param [in] armijoDescentMetric : The step Armijo descent metric defined as dc/dw' * delta_w
   */
  std::pair<bool, StepType> acceptStep(const PerformanceIndex& baselinePerformance, const PerformanceIndex& stepPerformance,
                                       scalar_t armijoDescentMetric) const;

  /** Compute total constraint violation */
  static scalar_t totalConstraintViolation(const PerformanceIndex& performance) {
    return std::sqrt(performance.dynamicsViolationSSE + performance.equalityConstraintsSSE);
  }
};

/** Transforms the StepType to string */
std::string toString(const FilterLinesearch::StepType& stepType);

/**
 * Computes the Armijo descent metric that determines if the solution is a descent direction for the cost. It calculates sum of
 * gradient(cost).dot([dx; du]) over the trajectory.
 *
 * @param [in] cost: The quadratic approximation of the cost.
 * @param [in] deltaXSol: The state trajectory of the QP subproblem solution.
 * @param [in] deltaUSol: The input trajectory of the QP subproblem solution.
 * @return The Armijo descent metric.
 */
scalar_t armijoDescentMetric(const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& deltaXSol,
                             const vector_array_t& deltaUSol);

}  // namespace ocs2
