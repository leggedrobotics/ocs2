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
#include <ostream>

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Defines the performance indices for a rollout
 */
struct PerformanceIndex {
  /** The merit function of a rollout. */
  scalar_t merit = 0.0;
  /** The total cost of a rollout. */
  scalar_t totalCost = 0.0;
  /** The integral of squared error for intermediate state-only equality constraints. */
  scalar_t stateEqConstraintISE = 0.0;
  /** The sum of squared error for intermediate state-only equality constraints. */
  scalar_t stateEqFinalConstraintSSE = 0.0;
  /** The integral of squared error for intermediate state-input equality constraints. */
  scalar_t stateInputEqConstraintISE = 0.0;
  /** The integral of squared error for intermediate inequality constraints violation. */
  scalar_t inequalityConstraintISE = 0.0;
  /** The total penalty of the intermediate inequality constraints violation. */
  scalar_t inequalityConstraintPenalty = 0.0;

  /** Add performance indices */
  PerformanceIndex& operator+=(const PerformanceIndex& rhs) {
    this->merit += rhs.merit;
    this->totalCost += rhs.totalCost;
    this->stateEqConstraintISE += rhs.stateEqConstraintISE;
    this->stateEqFinalConstraintSSE += rhs.stateEqFinalConstraintSSE;
    this->stateInputEqConstraintISE += rhs.stateInputEqConstraintISE;
    this->inequalityConstraintISE += rhs.inequalityConstraintISE;
    this->inequalityConstraintPenalty += rhs.inequalityConstraintPenalty;
    return *this;
  }
};

inline PerformanceIndex operator+(PerformanceIndex lhs, const PerformanceIndex& rhs) {
  lhs += rhs;  // Copied lhs, add rhs to it.
  return lhs;
}

inline std::ostream& operator<<(std::ostream& stream, const PerformanceIndex& performanceIndex) {
  const size_t tabSpace = 12;
  const auto indentation = stream.width();
  stream << std::left;  // fill from left

  stream << std::setw(indentation) << "";
  stream << "rollout merit:                        " << std::setw(tabSpace) << performanceIndex.merit;
  stream << "rollout cost:                         " << std::setw(tabSpace) << performanceIndex.totalCost << '\n';

  stream << std::setw(indentation) << "";
  stream << "state equality constraints ISE:       " << std::setw(tabSpace) << performanceIndex.stateEqConstraintISE;
  stream << "state-input equality constraints ISE: " << std::setw(tabSpace) << performanceIndex.stateInputEqConstraintISE << '\n';

  stream << std::setw(indentation) << "";
  stream << "inequality constraints ISE:           " << std::setw(tabSpace) << performanceIndex.inequalityConstraintISE;
  stream << "inequality constraints penalty:       " << std::setw(tabSpace) << performanceIndex.inequalityConstraintPenalty << '\n';

  stream << std::setw(indentation) << "";
  stream << "state equality final constraints SSE: " << std::setw(tabSpace) << performanceIndex.stateEqFinalConstraintSSE;

  return stream;
}

}  // namespace ocs2
