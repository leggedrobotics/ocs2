//
// Created by rgrandia on 15.03.22.
//

#pragma once

namespace convex_plane_decomposition {

/**
 * Timer class that can be repeatedly started and stopped. Statistics are collected for all measured intervals .
 *
 * Taken and adapted from ocs2_core
 */
class Timer {
 public:
  Timer()
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
  double getTotalInMilliseconds() const { return std::chrono::duration<double, std::milli>(totalTime_).count(); }

  /**
   * @return Maximum duration of a single interval
   */
  double getMaxIntervalInMilliseconds() const { return std::chrono::duration<double, std::milli>(maxIntervalTime_).count(); }

  /**
   * @return Duration of the last timed interval
   */
  double getLastIntervalInMilliseconds() const { return std::chrono::duration<double, std::milli>(lastIntervalTime_).count(); }

  /**
   * @return Average duration of all timed intervals
   */
  double getAverageInMilliseconds() const { return getTotalInMilliseconds() / numTimedIntervals_; }

 private:
  int numTimedIntervals_;
  std::chrono::nanoseconds totalTime_;
  std::chrono::nanoseconds maxIntervalTime_;
  std::chrono::nanoseconds lastIntervalTime_;
  std::chrono::steady_clock::time_point startTime_;
};

}  // namespace convex_plane_decomposition