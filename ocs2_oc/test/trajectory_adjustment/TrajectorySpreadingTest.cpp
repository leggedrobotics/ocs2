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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreading.h>

struct Result {
  ocs2::scalar_array_t timeTrajectory;
  ocs2::vector_array_t stateTrajectory;
  ocs2::vector_array_t inputTrajectory;
  ocs2::size_array_t postEventsIndeces;
  ocs2::vector_array_t eventDataArray;
  // for testing
  ocs2::size_array_t modeTrajectory;
  ocs2::size_array_t preEventModeTrajectory;
};

std::size_t modeAtTime(const ocs2::ModeSchedule& modeSchedule, ocs2::scalar_t time, bool isFinalTime) {
  if (isFinalTime) {
    auto timeItr = std::upper_bound(modeSchedule.eventTimes.begin(), modeSchedule.eventTimes.end(), time);
    int modeIndex = std::distance(modeSchedule.eventTimes.begin(), timeItr);
    return modeSchedule.modeSequence[modeIndex];
  } else {
    return modeSchedule.modeAtTime(time);
  }
}

class TrajectorySpreadingTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr bool DEBUG_PRINT = false;

  /******************************************************************************************************/
  TrajectorySpreadingTest() {
    // rollout settings
    const auto rolloutSettings = []() {
      ocs2::rollout::Settings rolloutSettings;
      rolloutSettings.absTolODE = 1e-8;
      rolloutSettings.relTolODE = 1e-6;
      rolloutSettings.maxNumStepsPerSecond = 10000;
      return rolloutSettings;
    }();

    // dynamics and rollout
    const ocs2::matrix_t A = (ocs2::matrix_t(STATE_DIM, STATE_DIM) << -2.0, -1.0, 1.0, 0.0).finished();
    const ocs2::matrix_t B = (ocs2::matrix_t(STATE_DIM, INPUT_DIM) << 1.0, 0.0).finished();
    const ocs2::matrix_t G = (ocs2::matrix_t(STATE_DIM, STATE_DIM) << 1.0, 0.0, 0.0, 0.9).finished();
    systemPtr.reset(new ocs2::LinearSystemDynamics(A, B, G));
    rolloutPtr.reset(new ocs2::TimeTriggeredRollout(*systemPtr, rolloutSettings));

    // controller
    const ocs2::scalar_array_t cntTimeStamp{timeInterval.first, timeInterval.second};
    const ocs2::vector_array_t uff(2, ocs2::vector_t::Ones(INPUT_DIM));
    const ocs2::matrix_array_t k(2, ocs2::matrix_t::Zero(INPUT_DIM, STATE_DIM));
    controllerPtr.reset(new ocs2::LinearController(cntTimeStamp, uff, k));

    // trajectory spreading
    trajectorySpreadingPtr.reset(new ocs2::TrajectorySpreading(true));
  }

  /******************************************************************************************************/
  Result rollout(const ocs2::ModeSchedule& modeSchedule, const std::pair<ocs2::scalar_t, ocs2::scalar_t>& timeInterval) const {
    Result out;
    ocs2::ModeSchedule modeScheduleTemp = modeSchedule;  // since it is TimeTriggeredRollout, it will not be changed
    rolloutPtr->run(timeInterval.first, initState, timeInterval.second, controllerPtr.get(), modeScheduleTemp, out.timeTrajectory,
                    out.postEventsIndeces, out.stateTrajectory, out.inputTrajectory);

    out.eventDataArray.resize(out.postEventsIndeces.size());
    // store pre-event state to event data array
    std::transform(out.postEventsIndeces.begin(), out.postEventsIndeces.end(), out.eventDataArray.begin(),
                   [&out](size_t eventIndex) { return out.stateTrajectory[eventIndex - 1]; });

    out.modeTrajectory.resize(out.timeTrajectory.size());
    for (int i = 0; i < out.modeTrajectory.size(); i++) {
      out.modeTrajectory[i] = modeAtTime(modeSchedule, out.timeTrajectory[i], i == out.modeTrajectory.size() - 1 ? true : false);
    }

    out.preEventModeTrajectory.resize(out.postEventsIndeces.size());
    std::transform(out.postEventsIndeces.begin(), out.postEventsIndeces.end(), out.preEventModeTrajectory.begin(),
                   [&out, &modeSchedule](size_t eventIndex) { return modeSchedule.modeAtTime(out.timeTrajectory[eventIndex - 1]); });
    return out;
  }

  /******************************************************************************************************/
  std::pair<Result, ocs2::TrajectorySpreading::Status> spreadRollout(const ocs2::ModeSchedule& modeSchedule,
                                                                     const ocs2::ModeSchedule& updatedModeSchedule, const Result& in) {
    // set
    const auto status = trajectorySpreadingPtr->set(modeSchedule, updatedModeSchedule, in.timeTrajectory);

    Result out = in;
    trajectorySpreadingPtr->adjustTimeTrajectory(out.timeTrajectory);
    out.postEventsIndeces = trajectorySpreadingPtr->getPostEventIndices();
    trajectorySpreadingPtr->adjustTrajectory<ocs2::vector_t>(out.stateTrajectory);
    trajectorySpreadingPtr->adjustTrajectory<ocs2::vector_t>(out.inputTrajectory);
    trajectorySpreadingPtr->adjustTrajectory(out.modeTrajectory);
    out.eventDataArray = trajectorySpreadingPtr->extractEventsArray(out.eventDataArray);
    out.preEventModeTrajectory = trajectorySpreadingPtr->extractEventsArray(out.preEventModeTrajectory);

    return {out, status};
  }

  /******************************************************************************************************/
  ocs2::TrajectorySpreading::Status checkResults(const ocs2::ModeSchedule& modeSchedule, const ocs2::ModeSchedule& updatedModeSchedule,
                                                 const std::pair<ocs2::scalar_t, ocs2::scalar_t>& period, bool display = true) {
    // rollout
    const auto result = rollout(modeSchedule, period);

    if (display) {
      std::cerr << "\nBefore TrajectorySpreading: \n";
      auto itr = result.postEventsIndeces.begin();
      for (size_t k = 0; k < result.timeTrajectory.size(); k++) {
        std::cerr << "\033[0;33m[" << std::setw(2) << k << "]\033[0m";
        std::cerr << "\033[0;34m(" << std::setw(2) << result.modeTrajectory[k] << ")\033[0m";

        std::cerr << "state [" << result.timeTrajectory[k] << "]: " << result.stateTrajectory[k].transpose() << "\n";
        if (itr != result.postEventsIndeces.end() && *itr == k + 1) {
          size_t eventInd = ocs2::lookup::findIndexInTimeArray(modeSchedule.eventTimes, result.timeTrajectory[k]);
          size_t preMode = modeSchedule.modeSequence[eventInd];
          size_t postMode = modeSchedule.modeSequence[eventInd + 1];
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          std::cerr << "Event time: " << result.timeTrajectory[*itr] << "(" << preMode << "->" << postMode << ")"
                    << "\n";
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          itr++;
        }
      }  // end of k loop
      std::cerr << std::endl;
    }

    // spread rollout
    Result spreadResult;
    ocs2::TrajectorySpreading::Status status;
    std::tie(spreadResult, status) = spreadRollout(modeSchedule, updatedModeSchedule, result);

    if (display) {
      std::cerr << "\nAfter TrajectorySpreading: \n";
      auto itr = spreadResult.postEventsIndeces.begin();
      for (size_t k = 0; k < spreadResult.timeTrajectory.size(); k++) {
        std::cerr << "\033[0;33m[" << std::setw(2) << k << "]\033[0m";
        std::cerr << "\033[0;34m(" << std::setw(2) << spreadResult.modeTrajectory[k] << ")\033[0m";

        std::cerr << "state [" << spreadResult.timeTrajectory[k] << "]: " << spreadResult.stateTrajectory[k].transpose() << "\n";
        if (itr != spreadResult.postEventsIndeces.end() && *itr == k + 1) {
          size_t eventInd = ocs2::lookup::findIndexInTimeArray(updatedModeSchedule.eventTimes, spreadResult.timeTrajectory[k]);
          size_t preMode = updatedModeSchedule.modeSequence[eventInd];
          size_t postMode = updatedModeSchedule.modeSequence[eventInd + 1];
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          std::cerr << "Event time: " << spreadResult.timeTrajectory[*itr] << "(" << preMode << "->" << postMode << ")"
                    << "\n";
          std::cerr << "+++++++++++++++++++++++++++++++++++++\n";
          itr++;
        }
      }  // end of k loop
      std::cerr << std::endl;
    }

    if (display) {
      auto print = [](size_t n) { std::cerr << n << ", "; };
      std::cerr << "Post Event Indices: ";
      std::for_each(spreadResult.postEventsIndeces.begin(), spreadResult.postEventsIndeces.end(), print);
      std::cerr << "\n";
    }

    // test postEventsIndeces
    if (!spreadResult.timeTrajectory.empty()) {
      const ocs2::scalar_t initTime = spreadResult.timeTrajectory.front();
      const ocs2::scalar_t finalTime = spreadResult.timeTrajectory.back();

      const auto startEventItr = std::upper_bound(updatedModeSchedule.eventTimes.begin(), updatedModeSchedule.eventTimes.end(), initTime);
      // If the final time is aligned with(the same as) an event time, it belongs to the post-mode.
      const auto endEventItr = std::upper_bound(updatedModeSchedule.eventTimes.begin(), updatedModeSchedule.eventTimes.end(), finalTime);

      ocs2::size_array_t postEventIndices(endEventItr - startEventItr);
      for (auto itr = startEventItr; itr != endEventItr; itr++) {
        int index = std::distance(startEventItr, itr);
        if (itr == endEventItr - 1 && *itr == spreadResult.timeTrajectory.back()) {
          postEventIndices[index] = spreadResult.timeTrajectory.size() - 1;
        } else {
          auto timeItr = std::upper_bound(spreadResult.timeTrajectory.begin(), spreadResult.timeTrajectory.end(), *itr);
          postEventIndices[index] = std::distance(spreadResult.timeTrajectory.begin(), timeItr);
        }
      }

      EXPECT_TRUE(spreadResult.postEventsIndeces.size() == postEventIndices.size());
      EXPECT_TRUE(std::equal(postEventIndices.begin(), postEventIndices.end(), spreadResult.postEventsIndeces.begin()));

    } else {
      EXPECT_EQ(spreadResult.postEventsIndeces.size(), 0);
    }
    // test postEventsIndeces end

    // test timeTrajectory & stateTrajectory
    // initial time should NOT be changed as long as the time trajectory is non-empty
    if (!spreadResult.timeTrajectory.empty()) {
      EXPECT_TRUE(std::abs(spreadResult.timeTrajectory.front() - period.first) < ocs2::numeric_traits::limitEpsilon<ocs2::scalar_t>());
    }

    auto eventIndexActualItr = spreadResult.postEventsIndeces.begin();
    auto eventTimeReferenceInd = ocs2::lookup::findIndexInTimeArray(updatedModeSchedule.eventTimes, period.first);
    for (size_t k = 0; k < spreadResult.timeTrajectory.size(); k++) {
      // Time should be monotonic sequence except the final time. It is possible that the last two time points have the same time, but one
      // stands for pre-mode and the other stands for post-mode.
      if (k > 0 && k < spreadResult.timeTrajectory.size() - 1) {
        EXPECT_TRUE(spreadResult.timeTrajectory[k - 1] < spreadResult.timeTrajectory[k]) << "TimeIndex: " << k;
      }

      // Pre-event time should be equal to the event time
      if (eventIndexActualItr != spreadResult.postEventsIndeces.end() && *eventIndexActualItr == k + 1) {
        constexpr auto eps = ocs2::numeric_traits::limitEpsilon<ocs2::scalar_t>();
        EXPECT_EQ(spreadResult.timeTrajectory[k], updatedModeSchedule.eventTimes[eventTimeReferenceInd]);

        // when there is only one point in a mode(for example mode A), the next mode(for example mode B) will overwrite the post-event time
        // of mode A, and thus the time differene between post-event and  pre-event is NOT necessary to be within the limit epsilon
        // EXPECT_LT(spreadResult.timeTrajectory[k + 1], updatedModeSchedule.eventTimes[eventTimeReferenceInd] + eps);

        eventTimeReferenceInd++;
        eventIndexActualItr++;
      }
      // mode should match the given modeSchedule
      auto updatedMode =
          modeAtTime(updatedModeSchedule, spreadResult.timeTrajectory[k], k == spreadResult.timeTrajectory.size() - 1 ? true : false);
      EXPECT_TRUE(updatedMode == spreadResult.modeTrajectory[k]);
    }  // end of k loop

    // test postEventsIndeces
    EXPECT_TRUE(spreadResult.postEventsIndeces.size() == spreadResult.eventDataArray.size());
    for (int k = 0; k < spreadResult.postEventsIndeces.size(); k++) {
      const size_t preEventIndex = spreadResult.postEventsIndeces[k] - 1;
      const ocs2::scalar_t preEventTime = spreadResult.timeTrajectory[preEventIndex];
      // When spreading event time backward, the pre-event may change and differ from whatever that are stored in the eventDataArray. In
      // this situation, the following check will fail.
      // EXPECT_TRUE(spreadResult.stateTrajectory[preEventIndex].isApprox(spreadResult.eventDataArray[k])) << "Event #" << k << " is wrong";
      EXPECT_EQ(updatedModeSchedule.modeAtTime(preEventTime), spreadResult.preEventModeTrajectory[k]);
    }

    return status;
  }

  /******************************************************************************************************/
  const ocs2::vector_t initState = (ocs2::vector_t(STATE_DIM) << 0.0, 2.0).finished();
  const std::pair<ocs2::scalar_t, ocs2::scalar_t> timeInterval{0.0, 1.0};

  std::unique_ptr<ocs2::SystemDynamicsBase> systemPtr;
  std::unique_ptr<ocs2::TimeTriggeredRollout> rolloutPtr;
  std::unique_ptr<ocs2::LinearController> controllerPtr;
  std::unique_ptr<ocs2::TrajectorySpreading> trajectorySpreadingPtr;
};

constexpr size_t TrajectorySpreadingTest::STATE_DIM;
constexpr size_t TrajectorySpreadingTest::INPUT_DIM;
constexpr bool TrajectorySpreadingTest::DEBUG_PRINT;

TEST_F(TrajectorySpreadingTest, no_change) {
  const ocs2::scalar_array_t eventTimes{0.6, 1.7};
  const ocs2::size_array_t modeSequence{0, 1, 2};
  const ocs2::ModeSchedule& modeSchedule{eventTimes, modeSequence};
  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.0, 2};

  // rollout
  const auto result = rollout(modeSchedule, period);
  // spread rollout
  Result spreadResult;
  ocs2::TrajectorySpreading::Status status;
  std::tie(spreadResult, status) = spreadRollout(modeSchedule, modeSchedule, result);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
  EXPECT_TRUE(result.timeTrajectory == spreadResult.timeTrajectory);
  EXPECT_TRUE(result.stateTrajectory == spreadResult.stateTrajectory);
  EXPECT_TRUE(result.inputTrajectory == spreadResult.inputTrajectory);
  EXPECT_TRUE(result.postEventsIndeces == spreadResult.postEventsIndeces);
  EXPECT_TRUE(result.eventDataArray == spreadResult.eventDataArray);
}

TEST_F(TrajectorySpreadingTest, no_matching_modes) {
  const ocs2::scalar_array_t eventTimes{0.6, 1.7};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.0, 1.1};
  const ocs2::size_array_t updatedModeSequence{10, 11, 12};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.0, 2.0};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, partially_matching_modes) {
  const ocs2::scalar_array_t eventTimes{0.6, 1.7, 2.3};
  const ocs2::size_array_t modeSequence{0, 1, 2, 3};

  const ocs2::scalar_array_t updatedEventTimes{0.9, 1.1, 2.1};
  const ocs2::size_array_t updatedModeSequence{10, 1, 2, 30};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{1.0, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, final_time_is_the_same_as_event_time_1) {
  const ocs2::scalar_array_t eventTimes{1.1, 1.3};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.1, 2.1};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, final_time_is_the_same_as_event_time_2) {
  const ocs2::scalar_array_t eventTimes{1.1, 2.1};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.1, 1.3};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, erase_trajectory) {
  const ocs2::scalar_array_t eventTimes{1.1, 1.3};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.1, 1.3};
  const ocs2::size_array_t updatedModeSequence{0, 1, 3};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.2, 2.1};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_TRUE(status.willTruncate);
  EXPECT_FALSE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, fully_matched_modes) {
  const ocs2::scalar_array_t eventTimes{1.1, 1.3};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{0.5, 2.1};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.0, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);

  EXPECT_FALSE(status.willTruncate);
  EXPECT_TRUE(status.willPerformTrajectorySpreading);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_at_back_1) {
  const ocs2::scalar_array_t eventTimes{0.6, 1.7};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.0, 1.1};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.0, 1.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_at_back_2) {
  const ocs2::scalar_array_t eventTimes{1.0};
  const ocs2::size_array_t modeSequence{0, 1};

  const ocs2::scalar_array_t updatedEventTimes{1.0, 2.0};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_at_back_1) {
  const ocs2::scalar_array_t eventTimes{1.0, 1.1};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{0.6, 1.7};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.0, 1.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_at_back_2) {
  const ocs2::scalar_array_t eventTimes{1, 2, 3.1};
  const ocs2::size_array_t modeSequence{0, 1, 2, 4};

  const ocs2::scalar_array_t updatedEventTimes{1, 3};
  const ocs2::size_array_t updatedModeSequence{0, 1, 3};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.5, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_in_front_1) {
  const ocs2::scalar_array_t eventTimes{1, 2};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{0.5, 1.6};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, in_range_event_to_out_range_in_front_2) {
  const ocs2::scalar_array_t eventTimes{1, 2};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1.6};
  const ocs2::size_array_t updatedModeSequence{1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_in_front_1) {
  const ocs2::scalar_array_t eventTimes{2};
  const ocs2::size_array_t modeSequence{1, 2};

  const ocs2::scalar_array_t updatedEventTimes{0.5, 1.6};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, out_range_event_to_in_range_in_front_2) {
  const ocs2::scalar_array_t eventTimes{0.5, 2};
  const ocs2::size_array_t modeSequence{0, 1, 2};

  const ocs2::scalar_array_t updatedEventTimes{1, 1.6};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.7, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, overlap_forward) {
  const ocs2::scalar_array_t eventTimes{1, 1.5, 2};
  const ocs2::size_array_t modeSequence{0, 1, 2, 3};

  const ocs2::scalar_array_t updatedEventTimes{1, 2.2, 2.3};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2, 3};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0.9, 2.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, overlap_backward) {
  const ocs2::scalar_array_t eventTimes{1, 2.2, 3};
  const ocs2::size_array_t modeSequence{0, 1, 2, 3};

  const ocs2::scalar_array_t updatedEventTimes{1, 1.5, 2};
  const ocs2::size_array_t updatedModeSequence{0, 1, 2, 3};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{0, 3.5};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}

TEST_F(TrajectorySpreadingTest, anymal_test) {
  const ocs2::scalar_array_t eventTimes{1.00001, 1.40001, 1.80001, 2.20001, 2.60001};
  const ocs2::size_array_t modeSequence{15, 15, 7, 14, 11, 13};

  const ocs2::scalar_array_t updatedEventTimes{1.51913, 1.91913, 2.31913, 2.71913, 3.11913};
  const ocs2::size_array_t updatedModeSequence{15, 7, 14, 11, 13, 7};

  const std::pair<ocs2::scalar_t, ocs2::scalar_t> period{1.4, 2.4};
  const auto status = checkResults({eventTimes, modeSequence}, {updatedEventTimes, updatedModeSequence}, period);
}
