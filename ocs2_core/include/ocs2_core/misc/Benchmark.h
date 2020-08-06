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

#include <chrono>

#include "ocs2_core/Types.h"

namespace ocs2 {
namespace benchmark {

/**
 * Timer class that can be repeatedly started and stopped. Statistics are collected for all measured intervals .
 */
class RepeatedTimer {
 public:
  RepeatedTimer()
      : numTimedIntervals_(0),
        totalTime_(std::chrono::nanoseconds::zero()),
        maxIntervalTime_(std::chrono::nanoseconds::zero()),
        lastIntervalTime_(std::chrono::nanoseconds::zero()),
        startTime_(std::chrono::steady_clock::now()) {}

  /**
   *  Reset the timer statistics
   */
  void reset() {
    totalTime_ = std::chrono::nanoseconds::zero();
    maxIntervalTime_ = std::chrono::nanoseconds::zero();
    lastIntervalTime_ = std::chrono::nanoseconds::zero();
    numTimedIntervals_ = 0;
  }

  /**
   *  Start timing an interval
   */
  void startTimer() { startTime_ = std::chrono::steady_clock::now(); }

  /**
   * Stop timing of an interval
   */
  void endTimer() {
    auto endTime = std::chrono::steady_clock::now();
    lastIntervalTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(endTime - startTime_);
    maxIntervalTime_ = std::max(maxIntervalTime_, lastIntervalTime_);
    totalTime_ += lastIntervalTime_;
    numTimedIntervals_++;
  };

  /**
   * @return Number of intervals that were timed
   */
  int getNumTimedIntervals() const { return numTimedIntervals_; }

  /**
   * @return Total cumulative time of timed intervals
   */
  scalar_t getTotalInMilliseconds() const { return std::chrono::duration<scalar_t, std::milli>(totalTime_).count(); }

  /**
   * @return Maximum duration of a single interval
   */
  scalar_t getMaxIntervalInMilliseconds() const { return std::chrono::duration<scalar_t, std::milli>(maxIntervalTime_).count(); }

  /**
   * @return Duration of the last timed interval
   */
  scalar_t getLastIntervalInMilliseconds() const { return std::chrono::duration<scalar_t, std::milli>(lastIntervalTime_).count(); }

  /**
   * @return Average duration of all timed intervals
   */
  scalar_t getAverageInMilliseconds() const { return getTotalInMilliseconds() / numTimedIntervals_; }

 private:
  int numTimedIntervals_;
  std::chrono::nanoseconds totalTime_;
  std::chrono::nanoseconds maxIntervalTime_;
  std::chrono::nanoseconds lastIntervalTime_;
  std::chrono::steady_clock::time_point startTime_;
};

}  // namespace benchmark
}  // namespace ocs2
