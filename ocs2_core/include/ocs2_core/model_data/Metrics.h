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

/** The structure contains a term's constraint vector and its associated penalty */
struct LagrangianMetrics {
  LagrangianMetrics() : LagrangianMetrics(0.0, vector_t()) {}
  LagrangianMetrics(scalar_t penaltyArg, vector_t constraintArg) : penalty(penaltyArg), constraint(std::move(constraintArg)) {}

  scalar_t penalty;
  vector_t constraint;
};

/** A const reference view to LagrangianMetrics. This is useful for having an array of references to LagrangianMetrics. */
struct LagrangianMetricsConstRef {
  LagrangianMetricsConstRef(const LagrangianMetrics& metricsArg) : penalty(metricsArg.penalty), constraint(metricsArg.constraint) {}
  operator LagrangianMetrics() const { return {penalty, constraint}; }

  const scalar_t& penalty;
  const vector_t& constraint;
};

/**
 * The collection of cost, dynamics violation, constraints, and LagrangianMetrics structure for all possible constraint
 * terms (handled by Lagrangian method) in a point of time.
 *     cost : The total cost in a particular time point.
 *     dynamicsViolation : The vector of dynamics violation.
 *     stateEqConstraint : An array of all state equality constraints.
 *     stateInputEqConstraint : An array of all state-input equality constraints.
 *     stateIneqConstraint : An array of all state inequality constraints.
 *     stateInputIneqConstraint : An array of all state-input inequality constraints.
 *     stateEqLagrangian : An array of state equality constraint terms handled by Lagrangian method.
 *     stateIneqLagrangian : An array of state inequality constraint terms handled by Lagrangian method.
 *     stateInputEqLagrangian : An array of state-input equality constraint terms handled by Lagrangian method.
 *     stateInputIneqLagrangian : An array of state-input inequality constraint terms handled by Lagrangian method.
 */
struct Metrics {
  // Cost
  scalar_t cost;

  // Dynamics violation
  vector_t dynamicsViolation;

  // Equality constraints
  vector_array_t stateEqConstraint;
  vector_array_t stateInputEqConstraint;

  // Inequality constraints
  vector_array_t stateIneqConstraint;
  vector_array_t stateInputIneqConstraint;

  // Lagrangians
  std::vector<LagrangianMetrics> stateEqLagrangian;
  std::vector<LagrangianMetrics> stateIneqLagrangian;
  std::vector<LagrangianMetrics> stateInputEqLagrangian;
  std::vector<LagrangianMetrics> stateInputIneqLagrangian;

  /** Exchanges the values of Metrics */
  void swap(Metrics& other);

  /** Clears the value of the Metrics */
  void clear();

  /** Returns true if *this is approximately equal to other, within the precision determined by prec. */
  bool isApprox(const Metrics& other, scalar_t prec = 1e-8) const;
};

/** Sums penalties of an array of LagrangianMetrics. */
inline scalar_t sumPenalties(const std::vector<LagrangianMetrics>& metricsArray) {
  scalar_t s = 0.0;
  std::for_each(metricsArray.begin(), metricsArray.end(), [&s](const LagrangianMetrics& m) { s += m.penalty; });
  return s;
}

/** Computes the sum of squared norm of constraints of an array of LagrangianMetrics. */
inline scalar_t constraintsSquaredNorm(const std::vector<LagrangianMetrics>& metricsArray) {
  scalar_t s = 0.0;
  std::for_each(metricsArray.begin(), metricsArray.end(), [&s](const LagrangianMetrics& m) { s += m.constraint.squaredNorm(); });
  return s;
}

/** Computes the sum of squared norm of a vector of equality constraints violation. */
inline scalar_t getEqConstraintsSSE(const vector_t& eqConstraint) {
  if (eqConstraint.size() == 0) {
    return 0.0;
  } else {
    return eqConstraint.squaredNorm();
  }
}

/** Computes the sum of squared norm of a vector of inequality constraints violation. */
inline scalar_t getIneqConstraintsSSE(const vector_t& ineqConstraint) {
  if (ineqConstraint.size() == 0) {
    return 0.0;
  } else {
    return ineqConstraint.cwiseMin(0.0).squaredNorm();
  }
}

/** Computes the sum of squared norm of an array of equality constraints violation. */
inline scalar_t getEqConstraintsSSE(const vector_array_t& eqConstraint) {
  scalar_t s = 0.0;
  std::for_each(eqConstraint.begin(), eqConstraint.end(), [&s](const vector_t& v) { s += getEqConstraintsSSE(v); });
  return s;
}

/** Computes the sum of squared norm of an array of inequality constraints violation. */
inline scalar_t getIneqConstraintsSSE(const vector_array_t& ineqConstraint) {
  scalar_t s = 0.0;
  std::for_each(ineqConstraint.begin(), ineqConstraint.end(), [&s](const vector_t& v) { s += getIneqConstraintsSSE(v); });
  return s;
}

/**
 * Serializes an array of LagrangianMetrics structures associated to an array of constraint terms.
 *
 * @ param [in] termsLagrangianMetrics : LagrangianMetrics associated to an array of constraint terms.
 * @return Serialized vector of the format : (..., termsMultiplier[i].penalty, termsMultiplier[i].constraint, ...).
 */
vector_t toVector(const std::vector<LagrangianMetrics>& termsLagrangianMetrics);

/**
 * Serializes an array of constraint terms.
 *
 * @ param [in] constraintArray : An array of constraint terms.
 * @return Serialized vector of the format : (...,  constraintArray[i], ...).
 */
vector_t toVector(const vector_array_t& constraintArray);

/**
 * Gets the size of constraint terms.
 *
 * @ param [in] constraintArray : An array of constraint terms.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t getSizes(const vector_array_t& constraintArray);

/**
 * Gets the size of constraint Lagrangian terms.
 *
 * @ param [in] termsLagrangianMetrics : LagrangianMetrics associated to an array of constraint terms.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t getSizes(const std::vector<LagrangianMetrics>& termsLagrangianMetrics);

/**
 * Deserializes the vector to an array of constraint terms.
 *
 * @param [in] termsSize : An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec : Serialized array of constraint terms of the format :
 *                   (..., constraintArray[i], ...)
 * @return An array of constraint terms.
 */
vector_array_t toConstraintArray(const size_array_t& termsSize, const vector_t& vec);

/**
 * Deserializes the vector to an array of LagrangianMetrics structures based on size of constraint terms.
 *
 * @param [in] termsSize : An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec : Serialized array of LagrangianMetrics structures of the format :
 *                   (..., termsMultiplier[i].penalty, termsMultiplier[i].constraint, ...)
 * @return An array of LagrangianMetrics structures associated to an array of constraint terms
 */
std::vector<LagrangianMetrics> toLagrangianMetrics(const size_array_t& termsSize, const vector_t& vec);

}  // namespace ocs2

namespace ocs2 {
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
 * Linearly interpolates a trajectory of Metrics.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of Metrics.
 * @return The interpolated Metrics at indexAlpha.
 */
Metrics interpolate(const index_alpha_t& indexAlpha, const std::vector<Metrics>& dataArray);

}  // namespace LinearInterpolation
}  // namespace ocs2
