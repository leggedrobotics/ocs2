/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_core/Types.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

struct LagrangianMetrics {
  LagrangianMetrics() : LagrangianMetrics(0.0, vector_t()) {}
  LagrangianMetrics(scalar_t penaltyArg, vector_t constraintArg) : penalty(penaltyArg), constraint(std::move(constraintArg)) {}

  scalar_t penalty;
  vector_t constraint;
};

/** A const reference view to LagrangianMetrics. This is useful for having an array of references to LagrangianMetrics. */
struct LagrangianMetricsConstRef {
  LagrangianMetricsConstRef(const LagrangianMetrics& metricsArg) : penalty(metricsArg.penalty), constraint(metricsArg.constraint) {}

  const scalar_t& penalty;
  const vector_t& constraint;
};

struct MetricsCollection {
  // Cost
  scalar_t cost;

  // Equality constraints
  vector_t stateEqConstraint;
  vector_t stateInputEqConstraint;

  // Lagrangians
  std::vector<LagrangianMetrics> stateEqLagrangian;
  std::vector<LagrangianMetrics> stateIneqLagrangian;
  std::vector<LagrangianMetrics> stateInputEqLagrangian;
  std::vector<LagrangianMetrics> stateInputIneqLagrangian;

  /** Exchanges the values of MetricsCollection */
  void swap(MetricsCollection& other) {
    // Cost
    std::swap(cost, other.cost);
    // Equality constraints
    stateEqConstraint.swap(other.stateEqConstraint);
    stateInputEqConstraint.swap(other.stateInputEqConstraint);
    // Lagrangians
    stateEqLagrangian.swap(other.stateEqLagrangian);
    stateIneqLagrangian.swap(other.stateIneqLagrangian);
    stateInputEqLagrangian.swap(other.stateInputEqLagrangian);
    stateInputIneqLagrangian.swap(other.stateInputIneqLagrangian);
  }

  /** Clears the value of the MetricsCollection */
  void clear() {
    // Cost
    cost = 0.0;
    // Equality constraints
    stateEqConstraint = vector_t();
    stateInputEqConstraint = vector_t();
    // Lagrangians
    stateEqLagrangian.clear();
    stateIneqLagrangian.clear();
    stateInputEqLagrangian.clear();
    stateInputIneqLagrangian.clear();
  }
};

/** Sums penalties of an array of LagrangianMetrics */
inline scalar_t sumPenalties(const std::vector<LagrangianMetrics>& metricsArray) {
  scalar_t s = 0.0;
  std::for_each(metricsArray.begin(), metricsArray.end(), [&s](const LagrangianMetrics& m) { s += m.penalty; });
  return s;
}

namespace LinearInterpolation {

/**
 * Linearly interpolates a trajectory of LagrangianMetrics.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of LagrangianMetricsConstRef.
 * @return The interpolated LagrangianMetrics at indexAlpha.
 */
LagrangianMetrics interpolate(const index_alpha_t& indexAlpha, const std::vector<LagrangianMetricsConstRef>& dataArray);

/**
 * Linearly interpolates a trajectory of MetricsCollection.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MetricsCollection.
 * @return The interpolated MetricsCollection at indexAlpha.
 */
MetricsCollection interpolate(const index_alpha_t& indexAlpha, const std::vector<MetricsCollection>& dataArray);

}  // namespace LinearInterpolation

}  // namespace ocs2
