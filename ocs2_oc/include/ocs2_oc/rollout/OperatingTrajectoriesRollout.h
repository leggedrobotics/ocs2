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

#include <memory>

#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

#include "RolloutBase.h"

namespace ocs2 {

/**
 * This class is an interface class for forward rollout of the system dynamics.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class OperatingTrajectoriesRollout : public RolloutBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = RolloutBase<STATE_DIM, INPUT_DIM>;

  using typename BASE::controller_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;
  using typename BASE::time_interval_array_t;

  using operating_trajectories_t = SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM>;

  /**
   * Constructor.
   *
   * @param [in] operatingTrajectories: The operating trajectories used for initialization.
   * @param [in] rolloutSettings: The rollout settings.
   */
  explicit OperatingTrajectoriesRollout(const operating_trajectories_t& operatingTrajectories,
                                        const Rollout_Settings& rolloutSettings = Rollout_Settings())
      : BASE(rolloutSettings), operatingTrajectoriesPtr_(operatingTrajectories.clone()) {}

  /**
   * Default destructor.
   */
  ~OperatingTrajectoriesRollout() override = default;

  OperatingTrajectoriesRollout<STATE_DIM, INPUT_DIM>* clone() const override {
    return new OperatingTrajectoriesRollout<STATE_DIM, INPUT_DIM>(*operatingTrajectoriesPtr_, this->settings());
  }

 protected:
  state_vector_t runImpl(time_interval_array_t timeIntervalArray, const state_vector_t& initState, controller_t* controller,
                         scalar_array_t& timeTrajectory, size_array_t& postEventIndicesStock, state_vector_array_t& stateTrajectory,
                         input_vector_array_t& inputTrajectory) override {
    const int numSubsystems = timeIntervalArray.size();
    const int numEvents = numSubsystems - 1;

    // clearing the output trajectories
    timeTrajectory.clear();
    timeTrajectory.reserve(2 * numSubsystems);
    stateTrajectory.clear();
    stateTrajectory.reserve(2 * numSubsystems);
    inputTrajectory.clear();
    inputTrajectory.reserve(2 * numSubsystems);
    postEventIndicesStock.clear();
    postEventIndicesStock.reserve(numEvents);

    state_vector_t beginState = initState;
    scalar_t beginTime, endTime;
    for (int i = 0; i < numSubsystems; i++) {
      // get operating trajectories
      operatingTrajectoriesPtr_->getSystemOperatingTrajectories(beginState, timeIntervalArray[i].first, timeIntervalArray[i].second,
                                                                timeTrajectory, stateTrajectory, inputTrajectory, true);

      if (i < numEvents) {
        postEventIndicesStock.push_back(stateTrajectory.size());
        beginState = stateTrajectory.back();
      }

    }  // end of i loop

    return stateTrajectory.back();
  }

 private:
  std::unique_ptr<operating_trajectories_t> operatingTrajectoriesPtr_;
};

}  // namespace ocs2
