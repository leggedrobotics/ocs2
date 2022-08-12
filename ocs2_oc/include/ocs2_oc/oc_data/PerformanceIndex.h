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

#include <iomanip>
#include <limits>
#include <ostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/Metrics.h>

namespace ocs2 {

/**
 * Defines the performance indices for a rollout
 */
struct PerformanceIndex {
  /** The merit function of a rollout. */
  scalar_t merit = 0.0;

  /** The total cost of a rollout. */
  scalar_t cost = 0.0;

  /** Sum of Squared Error (SSE) of system dynamics violation */
  scalar_t dynamicsViolationSSE = 0.0;

  /** Sum of Squared Error (SSE) of equality constraints:
   * - Final: squared norm of violation in state equality constraints
   * - PreJumps: sum of squared norm of violation in state equality constraints
   * - Intermediates: Integral of squared norm violation in state/state-input equality constraints
   */
  scalar_t equalityConstraintsSSE = 0.0;

  /** Sum of equality Lagrangians:
   * - Final: penalty for violation in state equality constraints
   * - PreJumps: penalty for violation in state equality constraints
   * - Intermediates: penalty for violation in state/state-input equality constraints
   */
  scalar_t equalityLagrangian = 0.0;

  /** Sum of inequality Lagrangians:
   * - Final: penalty for violation in state inequality constraints
   * - PreJumps: penalty for violation in state inequality constraints
   * - Intermediates: penalty for violation in state/state-input inequality constraints
   */
  scalar_t inequalityLagrangian = 0.0;

  /** Add performance indices */
  PerformanceIndex& operator+=(const PerformanceIndex& rhs) {
    this->merit += rhs.merit;
    this->cost += rhs.cost;
    this->dynamicsViolationSSE += rhs.dynamicsViolationSSE;
    this->equalityConstraintsSSE += rhs.equalityConstraintsSSE;
    this->equalityLagrangian += rhs.equalityLagrangian;
    this->inequalityLagrangian += rhs.inequalityLagrangian;
    return *this;
  }

  /** Multiply by a scalar */
  template <typename SCALAR_T>
  PerformanceIndex& operator*=(const SCALAR_T c) {
    this->merit *= static_cast<scalar_t>(c);
    this->cost *= static_cast<scalar_t>(c);
    this->dynamicsViolationSSE *= static_cast<scalar_t>(c);
    this->equalityConstraintsSSE *= static_cast<scalar_t>(c);
    this->equalityLagrangian *= static_cast<scalar_t>(c);
    this->inequalityLagrangian *= static_cast<scalar_t>(c);
    return *this;
  }

  /** Returns true if *this is approximately equal to other, within the precision determined by prec. */
  bool isApprox(const PerformanceIndex other, const scalar_t prec = 1e-8) const {
    auto fuzzyCompares = [&](const scalar_t a, const scalar_t b) {
      return std::abs(a - b) <= prec * std::min(std::abs(a), std::abs(b)) || std::abs(a - b) < std::numeric_limits<scalar_t>::min();
    };
    bool isEqual = fuzzyCompares(this->merit, other.merit);
    isEqual = isEqual && fuzzyCompares(this->cost, other.cost);
    isEqual = isEqual && fuzzyCompares(this->dynamicsViolationSSE, other.dynamicsViolationSSE);
    isEqual = isEqual && fuzzyCompares(this->equalityConstraintsSSE, other.equalityConstraintsSSE);
    isEqual = isEqual && fuzzyCompares(this->equalityLagrangian, other.equalityLagrangian);
    isEqual = isEqual && fuzzyCompares(this->inequalityLagrangian, other.inequalityLagrangian);
    return isEqual;
  }
};

inline PerformanceIndex operator+(PerformanceIndex lhs, const PerformanceIndex& rhs) {
  lhs += rhs;  // copied lhs, add rhs to it.
  return lhs;
}

template <typename SCALAR_T>
PerformanceIndex operator*(PerformanceIndex lhs, const SCALAR_T c) {
  lhs *= c;  // copied lhs
  return lhs;
}

template <typename SCALAR_T>
PerformanceIndex operator*(const SCALAR_T c, PerformanceIndex rhs) {
  rhs *= c;  // copied rhs
  return rhs;
}

/** Swaps performance indices */
inline void swap(PerformanceIndex& lhs, PerformanceIndex& rhs) {
  std::swap(lhs.merit, rhs.merit);
  std::swap(lhs.cost, rhs.cost);
  std::swap(lhs.dynamicsViolationSSE, rhs.dynamicsViolationSSE);
  std::swap(lhs.equalityConstraintsSSE, rhs.equalityConstraintsSSE);
  std::swap(lhs.equalityLagrangian, rhs.equalityLagrangian);
  std::swap(lhs.inequalityLagrangian, rhs.inequalityLagrangian);
}

inline std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex) {
  const size_t tabSpace = 12;
  const auto indentation = stream.width();
  stream << std::left;  // fill from left

  stream << std::setw(indentation) << "";
  stream << "Rollout Merit:              " << std::setw(tabSpace) << performanceIndex.merit;
  stream << "Rollout Cost:               " << std::setw(tabSpace) << performanceIndex.cost << '\n';

  stream << std::setw(indentation) << "";
  stream << "Dynamics violation SSE:     " << std::setw(tabSpace) << performanceIndex.dynamicsViolationSSE;
  stream << "Equality constraints SSE:   " << std::setw(tabSpace) << performanceIndex.equalityConstraintsSSE << '\n';

  stream << std::setw(indentation) << "";
  stream << "Equality Lagrangian:        " << std::setw(tabSpace) << performanceIndex.equalityLagrangian;
  stream << "Inequality Lagrangian:      " << std::setw(tabSpace) << performanceIndex.inequalityLagrangian;

  return stream;
}

/** Computes the PerformanceIndex based on a given Metrics */
inline PerformanceIndex toPerformanceIndex(const Metrics& m) {
  PerformanceIndex performanceIndex;
  performanceIndex.merit = 0.0;  // left for the solver to fill
  performanceIndex.cost = m.cost;
  performanceIndex.dynamicsViolationSSE = m.dynamicsViolation.squaredNorm();
  performanceIndex.equalityConstraintsSSE = constraintsSquaredNorm(m.stateEqConstraint) + constraintsSquaredNorm(m.stateInputEqConstraint);
  performanceIndex.equalityLagrangian = sumPenalties(m.stateEqLagrangian) + sumPenalties(m.stateInputEqLagrangian);
  performanceIndex.inequalityLagrangian = sumPenalties(m.stateIneqLagrangian) + sumPenalties(m.stateInputIneqLagrangian);
  return performanceIndex;
}

}  // namespace ocs2
