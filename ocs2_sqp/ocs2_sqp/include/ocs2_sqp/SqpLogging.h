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

#include <ostream>
#include <string>

#include <ocs2_core/Types.h>

#include <ocs2_oc/oc_data/PerformanceIndex.h>

#include "ocs2_sqp/SqpSolverStatus.h"

namespace ocs2 {
namespace sqp {

struct LogEntry {
  // Problem info
  size_t problemNumber = 0;
  scalar_t time = 0.0;  // horizon start time for the currently solved problem

  // Iteration info
  size_t iteration = 0;

  // Computation time
  scalar_t linearQuadraticApproximationTime = 0.0;
  scalar_t solveQpTime = 0.0;
  scalar_t linesearchTime = 0.0;

  // Line search
  PerformanceIndex baselinePerformanceIndex;  // before taking the step
  scalar_t totalConstraintViolationBaseline;  // constraint metric used in the line search
  StepInfo stepInfo;

  // Convergence
  Convergence convergence = Convergence::FALSE;
};

std::ostream& operator<<(std::ostream& stream, const LogEntry& logEntry);

std::string logHeader();

template <typename T>
class Logger {
 public:
  explicit Logger(size_t numberOfEntries);

  T& currentEntry();

  void advance();

  void write(std::ostream& stream) const;

 private:
  using const_iterator = typename std::vector<T>::const_iterator;
  using iterator = typename std::vector<T>::iterator;

  bool full_;
  const_iterator front_;
  iterator back_;
  std::vector<T> data_;
};

template <typename T>
Logger<T>::Logger(size_t numberOfEntries)
    : full_(false),
      data_(numberOfEntries + 1, T())  // +1 so we have space for a new element to write to when full.
{
  front_ = data_.begin();
  back_ = data_.begin();
}

template <typename T>
T& Logger<T>::currentEntry() {
  return *back_;
}

template <typename T>
void Logger<T>::advance() {
  // move current entry iterator
  ++back_;

  // Wrap back pointer
  if (back_ == data_.end()) {
    back_ = data_.begin();
    full_ = true;
  }

  // Move (+wrap) front point
  if (full_) {
    ++front_;
    if (front_ == data_.end()) {
      front_ = data_.begin();
    }
  }
}

template <typename T>
void Logger<T>::write(std::ostream& stream) const {
  const_iterator writeIt = front_;

  if (writeIt > back_) {
    // write front -> end
    while (writeIt != data_.end()) {
      stream << *writeIt;
      ++writeIt;
    }
    // wrap around
    writeIt = data_.cbegin();
  }

  // write front -> back
  while (writeIt < back_) {
    stream << *writeIt;
    ++writeIt;
  }
}

}  // namespace sqp
}  // namespace ocs2