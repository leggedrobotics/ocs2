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

namespace ocs2 {
namespace ros_msg_conversions {

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <class ContainerAllocator, size_t STATE_DIM, size_t INPUT_DIM>
void createObservationMsg(const SystemObservation<STATE_DIM, INPUT_DIM>& observation,
                          ocs2_msgs::mpc_observation_<ContainerAllocator>& observationMsg) {
  observationMsg.time = observation.time();

  observationMsg.state.value.resize(STATE_DIM);
  for (size_t i = 0; i < STATE_DIM; i++) {
    observationMsg.state.value[i] = observation.state(i);
  }

  observationMsg.input.value.resize(INPUT_DIM);
  for (size_t i = 0; i < INPUT_DIM; i++) {
    observationMsg.input.value[i] = observation.input(i);
  }

  observationMsg.subsystem = observation.subsystem();
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <class ContainerAllocator, size_t STATE_DIM, size_t INPUT_DIM>
void readObservationMsg(const ocs2_msgs::mpc_observation_<ContainerAllocator>& observationMsg,
                        SystemObservation<STATE_DIM, INPUT_DIM>& observation) {
  using scalar_t = typename SystemObservation<STATE_DIM, INPUT_DIM>::scalar_t;
  observation.time() = observationMsg.time;

  observation.state() =
      Eigen::Map<const Eigen::Matrix<float, STATE_DIM, 1>>(observationMsg.state.value.data(), STATE_DIM).template cast<scalar_t>();

  observation.input() =
      Eigen::Map<const Eigen::Matrix<float, INPUT_DIM, 1>>(observationMsg.input.value.data(), INPUT_DIM).template cast<scalar_t>();

  observation.subsystem() = observationMsg.subsystem;
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <class ContainerAllocator>
void createModeScheduleMsg(const ModeSchedule& modeSchedule, ocs2_msgs::mode_schedule_<ContainerAllocator>& modeScheduleMsg) {
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
template <class ContainerAllocator>
ModeSchedule readModeScheduleMsg(const ocs2_msgs::mode_schedule_<ContainerAllocator>& modeScheduleMsg) {
  // event times
  std::vector<ModeSchedule::scalar_t> eventTimes;
  eventTimes.reserve(modeScheduleMsg.eventTimes.size());
  for (const auto& ti : modeScheduleMsg.eventTimes) {
    eventTimes.push_back(ti);
  }

  // mode sequence
  std::vector<size_t> modeSequence;
  modeSequence.reserve(modeScheduleMsg.modeSequence.size());
  for (const auto& si : modeScheduleMsg.modeSequence) {
    modeSequence.push_back(si);
  }

  return {eventTimes, modeSequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/***************************************************************************************************** */
template <class ContainerAllocator>
void createTargetTrajectoriesMsg(const CostDesiredTrajectories& costDesiredTrajectories,
                                 ocs2_msgs::mpc_target_trajectories_<ContainerAllocator>& targetTrajectoriesMsg) {
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
template <class ContainerAllocator>
void readTargetTrajectoriesMsg(const ocs2_msgs::mpc_target_trajectories_<ContainerAllocator>& targetTrajectoriesMsg,
                               CostDesiredTrajectories& costDesiredTrajectories) {
  using scalar_t = CostDesiredTrajectories::scalar_t;
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
                                    .template cast<scalar_t>();
  }  // end of i loop

  // input
  N = targetTrajectoriesMsg.inputTrajectory.size();
  desiredInputTrajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    desiredInputTrajectory[i] = Eigen::Map<const Eigen::VectorXf>(targetTrajectoriesMsg.inputTrajectory[i].value.data(),
                                                                  targetTrajectoriesMsg.inputTrajectory[i].value.size())
                                    .template cast<scalar_t>();
  }  // end of i loop
}

}  // namespace ros_msg_conversions
}  // namespace ocs2
