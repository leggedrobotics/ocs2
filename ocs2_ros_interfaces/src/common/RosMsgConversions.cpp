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

#include "ocs2_ros_interfaces/common/RosMsgConversions.h"

namespace ocs2 {
namespace ros_msg_conversions {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void createObservationMsg(const SystemObservation& observation, ocs2_msgs::mpc_observation& observationMsg) {
  observationMsg.time = observation.time;

  observationMsg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); i++) {
    observationMsg.state.value[i] = observation.state(i);
  }

  observationMsg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); i++) {
    observationMsg.input.value[i] = observation.input(i);
  }

  observationMsg.mode = observation.mode;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void readObservationMsg(const ocs2_msgs::mpc_observation& observationMsg, SystemObservation& observation) {
  observation.time = observationMsg.time;

  const auto& state = observationMsg.state.value;
  observation.state = Eigen::Map<const Eigen::VectorXf>(state.data(), state.size()).cast<scalar_t>();

  const auto& input = observationMsg.input.value;
  observation.input = Eigen::Map<const Eigen::VectorXf>(input.data(), input.size()).cast<scalar_t>();

  observation.mode = observationMsg.mode;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void createModeScheduleMsg(const ModeSchedule& modeSchedule, ocs2_msgs::mode_schedule& modeScheduleMsg) {
  // event times
  modeScheduleMsg.eventTimes.clear();
  modeScheduleMsg.eventTimes.reserve(modeSchedule.eventTimes.size());
  for (const auto& ti : modeSchedule.eventTimes) {
    modeScheduleMsg.eventTimes.push_back(ti);
  }

  // mode sequence
  modeScheduleMsg.modeSequence.clear();
  modeScheduleMsg.modeSequence.reserve(modeSchedule.modeSequence.size());
  for (const auto& si : modeSchedule.modeSequence) {
    modeScheduleMsg.modeSequence.push_back(si);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
ModeSchedule readModeScheduleMsg(const ocs2_msgs::mode_schedule& modeScheduleMsg) {
  // event times
  scalar_array_t eventTimes;
  eventTimes.reserve(modeScheduleMsg.eventTimes.size());
  for (const auto& ti : modeScheduleMsg.eventTimes) {
    eventTimes.push_back(ti);
  }

  // mode sequence
  size_array_t modeSequence;
  modeSequence.reserve(modeScheduleMsg.modeSequence.size());
  for (const auto& si : modeScheduleMsg.modeSequence) {
    modeSequence.push_back(si);
  }

  return {eventTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void createTargetTrajectoriesMsg(const CostDesiredTrajectories& costDesiredTrajectories,
                                 ocs2_msgs::mpc_target_trajectories& targetTrajectoriesMsg) {
  const auto& desiredTimeTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
  const auto& desiredStateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
  const auto& desiredInputTrajectory = costDesiredTrajectories.desiredInputTrajectory();

  // time and state
  size_t N = desiredStateTrajectory.size();
  targetTrajectoriesMsg.timeTrajectory.resize(N);
  targetTrajectoriesMsg.stateTrajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.timeTrajectory[i] = desiredTimeTrajectory[i];

    targetTrajectoriesMsg.stateTrajectory[i].value =
        std::vector<float>(desiredStateTrajectory[i].data(), desiredStateTrajectory[i].data() + desiredStateTrajectory[i].size());
  }  // end of i loop

  // input
  N = desiredInputTrajectory.size();
  targetTrajectoriesMsg.inputTrajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.inputTrajectory[i].value =
        std::vector<float>(desiredInputTrajectory[i].data(), desiredInputTrajectory[i].data() + desiredInputTrajectory[i].size());
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
void readTargetTrajectoriesMsg(const ocs2_msgs::mpc_target_trajectories& targetTrajectoriesMsg,
                               CostDesiredTrajectories& costDesiredTrajectories) {
  auto& desiredTimeTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
  auto& desiredStateTrajectory = costDesiredTrajectories.desiredStateTrajectory();
  auto& desiredInputTrajectory = costDesiredTrajectories.desiredInputTrajectory();

  size_t N = targetTrajectoriesMsg.stateTrajectory.size();
  if (N == 0) {
    throw std::runtime_error("An empty target trajectories message is received.");
  }

  // state and time
  desiredTimeTrajectory.resize(N);
  desiredStateTrajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    desiredTimeTrajectory[i] = targetTrajectoriesMsg.timeTrajectory[i];

    desiredStateTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(targetTrajectoriesMsg.stateTrajectory[i].value.data(),
                                                                  targetTrajectoriesMsg.stateTrajectory[i].value.size())
                                    .cast<scalar_t>();
  }  // end of i loop

  // input
  N = targetTrajectoriesMsg.inputTrajectory.size();
  desiredInputTrajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    desiredInputTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(targetTrajectoriesMsg.inputTrajectory[i].value.data(),
                                                                  targetTrajectoriesMsg.inputTrajectory[i].value.size())
                                    .cast<scalar_t>();
  }  // end of i loop
}

}  // namespace ros_msg_conversions
}  // namespace ocs2
