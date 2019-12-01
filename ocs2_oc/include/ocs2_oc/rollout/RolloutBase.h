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

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/control/ControllerBase.h>
#include "ocs2_core/misc/Numerics.h"

#include "Rollout_Settings.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class RolloutBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<RolloutBase<STATE_DIM, INPUT_DIM>>;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;

  using size_array_t = typename DIMENSIONS::size_array_t;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;

  using time_interval_t = std::pair<scalar_t, scalar_t>;
  using time_interval_array_t = std::vector<time_interval_t>;

  /**
   * Default constructor.
   *
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit RolloutBase(Rollout_Settings rolloutSettings = Rollout_Settings()) : rolloutSettings_(std::move(rolloutSettings)) {}

  /**
   * Default destructor.
   */
  virtual ~RolloutBase() = default;

  /**
   * Returns the rollout settings.
   *
   * @return The rollout settings.
   */
  Rollout_Settings& settings() { return rolloutSettings_; }

  /**
   * Returns the rollout settings.
   *
   * @return The rollout settings.
   */
  const Rollout_Settings& settings() const { return rolloutSettings_; }

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual RolloutBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] controller: control policy.
   * @param [in] eventTimes: The sorted event times array which can cover time beyond initTime and finalTime {s_0 < ... < s_{n-1}}.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] postEventIndicesStock: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   *
   * @return The final state (state jump is considered if it took place)
   */
  state_vector_t run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, controller_t* controller,
                     const scalar_array_t& eventTimes, scalar_array_t& timeTrajectory, size_array_t& postEventIndicesStock,
                     state_vector_array_t& stateTrajectory, input_vector_array_t& inputTrajectory) {
    if (initTime > finalTime) {
      throw std::runtime_error("Initial time should be less-equal to final time.");
    }

    // switching times
    auto firstIndex = std::upper_bound(eventTimes.begin(), eventTimes.end(), initTime);
    auto lastIndex = std::upper_bound(eventTimes.begin(), eventTimes.end(), finalTime);
    scalar_array_t switchingTimes;
    switchingTimes.push_back(initTime);
    switchingTimes.insert(switchingTimes.end(), firstIndex, lastIndex);
    switchingTimes.push_back(finalTime);

    // constructing the rollout time intervals
    time_interval_array_t timeIntervalArray;
    const int numSubsystems = switchingTimes.size() - 1;
    for (int i = 0; i < numSubsystems; i++) {
      const auto& beginTime = switchingTimes[i];
      const auto& endTime = switchingTimes[i + 1];
      timeIntervalArray.emplace_back(beginTime, endTime);

      // adjusting the start time for correcting the subsystem recognition
      const scalar_t eps = OCS2NumericTraits<scalar_t>::weakEpsilon();
      if (endTime - beginTime > eps) {
        timeIntervalArray.back().first += eps;
      } else {
        timeIntervalArray.back().first = endTime;
      }
    }  // end of for loop

    return runImpl(std::move(timeIntervalArray), initState, controller, timeTrajectory, postEventIndicesStock, stateTrajectory,
                   inputTrajectory);
  }

  /**
   * Prints out the rollout.
   *
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] postEventIndicesStock: An array of the post-event indices.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   */
  static void display(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndicesStock,
                      const state_vector_array_t& stateTrajectory, const input_vector_array_t* const inputTrajectory) {
    std::cerr << "Trajectory length:      " << timeTrajectory.size() << std::endl;
    std::cerr << "Total number of events: " << postEventIndicesStock.size() << std::endl;
    if (!postEventIndicesStock.empty()) {
      std::cerr << "Event times: ";
      for (size_t ind : postEventIndicesStock) {
        std::cerr << timeTrajectory[ind] << ", ";
      }
      std::cerr << std::endl;
    }
    std::cerr << std::endl;

    const size_t numSubsystems = postEventIndicesStock.size() + 1;
    size_t k = 0;
    for (size_t i = 0; i < numSubsystems; i++) {
      for (; k < timeTrajectory.size(); k++) {
        std::cerr << "Index: " << k << std::endl;
        std::cerr << "Time:  " << std::setprecision(12) << timeTrajectory[k] << std::endl;
        std::cerr << "State: " << std::setprecision(3) << stateTrajectory[k].transpose() << std::endl;
        if (inputTrajectory) {
          std::cerr << "Input: " << std::setprecision(3) << (*inputTrajectory)[k].transpose() << std::endl;
        }

        if (i < postEventIndicesStock.size() && k + 1 == postEventIndicesStock[i]) {
          std::cerr << "+++ event took place +++" << std::endl;
          k++;
          break;
        }
      }  // end of k loop
    }    // end of i loop
  }

 protected:
  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] timeIntervalArray: An array of the rollout's start and final times.
   * @param [in] initState: The initial state.
   * @param [in] controller: control policy.
   * @param [out] timeTrajectory: The time trajectory stamp.
   * @param [out] postEventIndicesStock: Indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectory: The state trajectory.
   * @param [out] inputTrajectory: The control input trajectory.
   *
   * @return The final state (state jump is considered if it took place)
   */
  virtual state_vector_t runImpl(time_interval_array_t timeIntervalArray, const state_vector_t& initState, controller_t* controller,
                                 scalar_array_t& timeTrajectory, size_array_t& postEventIndicesStock, state_vector_array_t& stateTrajectory,
                                 input_vector_array_t& inputTrajectory) = 0;

  /**
   * Checks for the numerical stability if Rollout_Settings::checkNumericalStability_ is true.
   *
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] postEventIndicesStock: Indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   */
  void checkNumericalStability(controller_t* controller, const scalar_array_t& timeTrajectory, const size_array_t& postEventIndicesStock,
                               const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory) const {
    if (!rolloutSettings_.checkNumericalStability_) {
      return;
    }

    for (size_t i = 0; i < timeTrajectory.size(); i++) {
      try {
        if (!stateTrajectory[i].allFinite()) {
          throw std::runtime_error("Rollout: state is not finite");
        }
        if (rolloutSettings_.reconstructInputTrajectory_ && !inputTrajectory[i].allFinite()) {
          throw std::runtime_error("Rollout: input is not finite");
        }
      } catch (const std::exception& error) {
        std::cerr << "what(): " << error.what() << " at time " + std::to_string(timeTrajectory[i]) + " [sec]." << std::endl;

        // truncate trajectories
        scalar_array_t timeTrajectoryTemp;
        state_vector_array_t stateTrajectoryTemp;
        input_vector_array_t inputTrajectoryTemp;
        for (size_t j = 0; j <= i; j++) {
          timeTrajectoryTemp.push_back(timeTrajectory[j]);
          stateTrajectoryTemp.push_back(stateTrajectory[j]);
          if (rolloutSettings_.reconstructInputTrajectory_) {
            inputTrajectoryTemp.push_back(inputTrajectory[j]);
          }
        }

        // display
        const input_vector_array_t* const inputTrajectoryTempPtr =
            rolloutSettings_.reconstructInputTrajectory_ ? &inputTrajectoryTemp : nullptr;
        display(timeTrajectoryTemp, postEventIndicesStock, stateTrajectoryTemp, inputTrajectoryTempPtr);

        controller->display();

        throw;
      }
    }  // end of i loop
  }

 private:
  Rollout_Settings rolloutSettings_;
};

}  // namespace ocs2
