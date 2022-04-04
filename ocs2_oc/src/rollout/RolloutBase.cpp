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

#include "ocs2_oc/rollout/RolloutBase.h"

#include <algorithm>
#include <iomanip>
#include <iostream>

#include <ocs2_core/NumericTraits.h>
#include <ocs2_core/misc/Numerics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<scalar_t, scalar_t>> RolloutBase::findActiveModesTimeInterval(scalar_t initTime, scalar_t finalTime,
                                                                                    const scalar_array_t& eventTimes) const {
  // switching times
  const auto firstIndex = std::upper_bound(eventTimes.cbegin(), eventTimes.cend(), initTime);  // no event at initial time
  const auto lastIndex = std::upper_bound(eventTimes.cbegin(), eventTimes.cend(), finalTime);  // can be an event at final time
  scalar_array_t switchingTimes;
  switchingTimes.push_back(initTime);
  switchingTimes.insert(switchingTimes.end(), firstIndex, lastIndex);
  switchingTimes.push_back(finalTime);

  // constructing the rollout time intervals
  const int numSubsystems = switchingTimes.size() - 1;  // switchingTimes contains at least two elements
  std::vector<std::pair<scalar_t, scalar_t>> timeIntervalArray(numSubsystems);
  for (int i = 0; i < numSubsystems; i++) {
    const auto& beginTime = switchingTimes[i];
    const auto& endTime = switchingTimes[i + 1];

    // adjusting the start time to correct for subsystem recognition
    constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();
    timeIntervalArray[i] = std::make_pair(std::min(beginTime + eps, endTime), endTime);
  }  // end of for loop

  return timeIntervalArray;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RolloutBase::display(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices, const vector_array_t& stateTrajectory,
                          const vector_array_t* const inputTrajectory) {
  std::cerr << "Trajectory length:      " << timeTrajectory.size() << '\n';
  std::cerr << "Total number of events: " << postEventIndices.size() << '\n';
  if (!postEventIndices.empty()) {
    std::cerr << "Event times: ";
    for (size_t ind : postEventIndices) {
      std::cerr << timeTrajectory[ind] << ", ";
    }
    std::cerr << '\n';
  }
  std::cerr << '\n';

  const size_t numSubsystems = postEventIndices.size() + 1;
  size_t k = 0;
  for (size_t i = 0; i < numSubsystems; i++) {
    for (; k < timeTrajectory.size(); k++) {
      std::cerr << "Index: " << k << '\n';
      std::cerr << "Time:  " << std::setprecision(12) << timeTrajectory[k] << '\n';
      std::cerr << "State: " << std::setprecision(3) << stateTrajectory[k].transpose() << '\n';
      if (inputTrajectory != nullptr) {
        std::cerr << "Input: " << std::setprecision(3) << (*inputTrajectory)[k].transpose() << '\n';
      }

      if (i < postEventIndices.size() && k + 1 == postEventIndices[i]) {
        std::cerr << "+++ event took place +++" << '\n';
        k++;
        break;
      }
    }  // end of k loop
  }    // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void RolloutBase::checkNumericalStability(const ControllerBase& controller, const scalar_array_t& timeTrajectory,
                                          const size_array_t& postEventIndices, const vector_array_t& stateTrajectory,
                                          const vector_array_t& inputTrajectory) const {
  if (!rolloutSettings_.checkNumericalStability) {
    return;
  }

  for (size_t i = 0; i < timeTrajectory.size(); i++) {
    try {
      if (!stateTrajectory[i].allFinite()) {
        throw std::runtime_error("Rollout: state is not finite");
      }
      if (rolloutSettings_.reconstructInputTrajectory && !inputTrajectory[i].allFinite()) {
        throw std::runtime_error("Rollout: input is not finite");
      }
    } catch (const std::exception& error) {
      std::cerr << "what(): " << error.what() << " at time " + std::to_string(timeTrajectory[i]) + " [sec]." << '\n';

      // truncate trajectories
      scalar_array_t timeTrajectoryTemp;
      vector_array_t stateTrajectoryTemp;
      vector_array_t inputTrajectoryTemp;
      for (size_t j = 0; j <= i; j++) {
        timeTrajectoryTemp.push_back(timeTrajectory[j]);
        stateTrajectoryTemp.push_back(stateTrajectory[j]);
        if (rolloutSettings_.reconstructInputTrajectory) {
          inputTrajectoryTemp.push_back(inputTrajectory[j]);
        }
      }

      // display
      const vector_array_t* const inputTrajectoryTempPtr = rolloutSettings_.reconstructInputTrajectory ? &inputTrajectoryTemp : nullptr;
      display(timeTrajectoryTemp, postEventIndices, stateTrajectoryTemp, inputTrajectoryTempPtr);

      controller.display();

      throw;
    }
  }  // end of i loop
}
}  // namespace ocs2
