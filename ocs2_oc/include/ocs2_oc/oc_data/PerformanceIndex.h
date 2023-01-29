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

  /** Sum of Squared Error (SSE) of the dual feasibilities:
   * - Final: squared norm of violation in the dual feasibilities
   * - PreJumps: sum of squared norm of violation in the dual feasibilities
   * - Intermediates: sum of squared norm of violation in the dual feasibilities
   */
  scalar_t dualFeasibilitiesSSE = 0.0;

  /** Sum of Squared Error (SSE) of system dynamics violation */
  scalar_t dynamicsViolationSSE = 0.0;

  /** Sum of Squared Error (SSE) of equality constraints:
   * - Final: squared norm of violation in state equality constraints
   * - PreJumps: sum of squared norm of violation in state equality constraints
   * - Intermediates: Integral of squared norm violation in state/state-input equality constraints
   */
  scalar_t equalityConstraintsSSE = 0.0;

  /** Sum of Squared Error (SSE) of inequality constraints:
   * - Final: squared norm of violation in state inequality constraints
   * - PreJumps: sum of squared norm of violation in state inequality constraints
   * - Intermediates: Integral of squared norm violation in state/state-input inequality constraints
   */
  scalar_t inequalityConstraintsSSE = 0.0;

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

  /** Add performance indices. */
  PerformanceIndex& operator+=(const PerformanceIndex& rhs);

  /** Multiply by a scalar. */
  PerformanceIndex& operator*=(const scalar_t c);

  /** Returns true if *this is approximately equal to other, within the precision determined by prec. */
  bool isApprox(const PerformanceIndex& other, const scalar_t prec = 1e-8) const;
};

/** Add performance indices. */
inline PerformanceIndex operator+(PerformanceIndex lhs, const PerformanceIndex& rhs) {
  lhs += rhs;  // copied lhs, add rhs to it.
  return lhs;
}

/** Multiply by a scalar. */
inline PerformanceIndex operator*(PerformanceIndex lhs, const scalar_t c) {
  lhs *= c;  // copied lhs
  return lhs;
}

/** Multiply by a scalar. */
inline PerformanceIndex operator*(const scalar_t c, PerformanceIndex rhs) {
  rhs *= c;  // copied rhs
  return rhs;
}

/** Swaps performance indices. */
void swap(PerformanceIndex& lhs, PerformanceIndex& rhs);

/** Computes the PerformanceIndex based on a given continuous-time Metrics. */
PerformanceIndex toPerformanceIndex(const Metrics& m);

/** Computes the PerformanceIndex based on a given discrete-time Metrics. */
PerformanceIndex toPerformanceIndex(const Metrics& m, const scalar_t dt);

/** Overloads the stream insertion operator. */
std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex);

}  // namespace ocs2
