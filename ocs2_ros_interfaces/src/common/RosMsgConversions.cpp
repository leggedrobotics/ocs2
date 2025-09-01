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
/******************************************************************************************************/
ocs2_msgs::msg::MpcObservation createObservationMsg(
    const SystemObservation& observation) {
  ocs2_msgs::msg::MpcObservation observationMsg;

  observationMsg.time = observation.time;

  observationMsg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); i++) {
    observationMsg.state.value[i] = static_cast<float>(observation.state(i));
  }

  observationMsg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); i++) {
    observationMsg.input.value[i] = static_cast<float>(observation.input(i));
  }

  observationMsg.mode = observation.mode;

  return observationMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SystemObservation readObservationMsg(
    const ocs2_msgs::msg::MpcObservation& observationMsg) {
  SystemObservation observation;

  observation.time = observationMsg.time;

  const auto& state = observationMsg.state.value;
  observation.state =
      Eigen::Map<const Eigen::VectorXf>(state.data(), state.size())
          .cast<scalar_t>();

  const auto& input = observationMsg.input.value;
  observation.input =
      Eigen::Map<const Eigen::VectorXf>(input.data(), input.size())
          .cast<scalar_t>();

  observation.mode = observationMsg.mode;

  return observation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::ModeSchedule createModeScheduleMsg(
    const ModeSchedule& modeSchedule) {
  ocs2_msgs::msg::ModeSchedule modeScheduleMsg;
  // event times
  modeScheduleMsg.event_times.clear();
  modeScheduleMsg.event_times.reserve(modeSchedule.eventTimes.size());
  for (const auto& ti : modeSchedule.eventTimes) {
    modeScheduleMsg.event_times.push_back(ti);
  }

  // mode sequence
  modeScheduleMsg.mode_sequence.clear();
  modeScheduleMsg.mode_sequence.reserve(modeSchedule.modeSequence.size());
  for (const auto& si : modeSchedule.modeSequence) {
    modeScheduleMsg.mode_sequence.push_back(si);
  }

  return modeScheduleMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSchedule readModeScheduleMsg(
    const ocs2_msgs::msg::ModeSchedule& modeScheduleMsg) {
  // event times
  scalar_array_t eventTimes;
  eventTimes.reserve(modeScheduleMsg.event_times.size());
  for (const auto& ti : modeScheduleMsg.event_times) {
    eventTimes.push_back(ti);
  }

  // mode sequence
  size_array_t mode_sequence;
  mode_sequence.reserve(modeScheduleMsg.mode_sequence.size());
  for (const auto& si : modeScheduleMsg.mode_sequence) {
    mode_sequence.push_back(si);
  }

  return {eventTimes, mode_sequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcPerformanceIndices createPerformanceIndicesMsg(
    scalar_t initTime,
    const PerformanceIndex& performanceIndices) {
  ocs2_msgs::msg::MpcPerformanceIndices performanceIndicesMsg;

  performanceIndicesMsg.init_time = initTime;
  performanceIndicesMsg.merit = performanceIndices.merit;
  performanceIndicesMsg.cost = performanceIndices.cost;
  performanceIndicesMsg.dynamics_violation_sse =
      performanceIndices.dynamicsViolationSSE;
  performanceIndicesMsg.equality_constraints_sse =
      performanceIndices.equalityConstraintsSSE;
  performanceIndicesMsg.equality_lagrangian =
      performanceIndices.equalityLagrangian;
  performanceIndicesMsg.inequality_lagrangian =
      performanceIndices.inequalityLagrangian;

  return performanceIndicesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex readPerformanceIndicesMsg(
    const ocs2_msgs::msg::MpcPerformanceIndices& performanceIndicesMsg) {
  PerformanceIndex performanceIndices;

  performanceIndices.merit = performanceIndicesMsg.merit;
  performanceIndices.cost = performanceIndicesMsg.cost;
  performanceIndices.dynamicsViolationSSE =
      performanceIndicesMsg.dynamics_violation_sse;
  performanceIndices.equalityConstraintsSSE =
      performanceIndicesMsg.equality_constraints_sse;
  performanceIndices.equalityLagrangian =
      performanceIndicesMsg.equality_lagrangian;
  performanceIndices.inequalityLagrangian =
      performanceIndicesMsg.inequality_lagrangian;

  return performanceIndices;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcTargetTrajectories createTargetTrajectoriesMsg(
    const TargetTrajectories& targetTrajectories) {
  ocs2_msgs::msg::MpcTargetTrajectories targetTrajectoriesMsg;
  const auto& timeTrajectory = targetTrajectories.timeTrajectory;
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // time and state
  size_t N = stateTrajectory.size();
  targetTrajectoriesMsg.time_trajectory.resize(N);
  targetTrajectoriesMsg.state_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.time_trajectory[i] = timeTrajectory[i];

    targetTrajectoriesMsg.state_trajectory[i].value = std::vector<float>(
        stateTrajectory[i].data(),
        stateTrajectory[i].data() + stateTrajectory[i].size());
  }  // end of i loop

  // input
  N = inputTrajectory.size();
  targetTrajectoriesMsg.input_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.input_trajectory[i].value = std::vector<float>(
        inputTrajectory[i].data(),
        inputTrajectory[i].data() + inputTrajectory[i].size());
  }  // end of i loop

  return targetTrajectoriesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories readTargetTrajectoriesMsg(
    const ocs2_msgs::msg::MpcTargetTrajectories& targetTrajectoriesMsg) {
  size_t N = targetTrajectoriesMsg.state_trajectory.size();
  if (N == 0) {
    throw std::runtime_error(
        "An empty target trajectories message is received.");
  }

  // state and time
  scalar_array_t desiredTimeTrajectory(N);
  vector_array_t desiredStateTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    desiredTimeTrajectory[i] = targetTrajectoriesMsg.time_trajectory[i];

    desiredStateTrajectory[i] =
        Eigen::Map<const Eigen::VectorXf>(
            targetTrajectoriesMsg.state_trajectory[i].value.data(),
            targetTrajectoriesMsg.state_trajectory[i].value.size())
            .cast<scalar_t>();
  }  // end of i loop

  // input
  N = targetTrajectoriesMsg.input_trajectory.size();
  vector_array_t desiredInputTrajectory(N);
  for (size_t i = 0; i < N; i++) {
    desiredInputTrajectory[i] =
        Eigen::Map<const Eigen::VectorXf>(
            targetTrajectoriesMsg.input_trajectory[i].value.data(),
            targetTrajectoriesMsg.input_trajectory[i].value.size())
            .cast<scalar_t>();
  }  // end of i loop

  return {desiredTimeTrajectory, desiredStateTrajectory,
          desiredInputTrajectory};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::Constraint createConstraintMsg(scalar_t time,
                                               const vector_t& constraint) {
  ocs2_msgs::msg::Constraint constraintMsg;

  constraintMsg.time = time;
  constraintMsg.value.resize(constraint.size());
  for (size_t i = 0; i < constraint.size(); i++) {
    constraintMsg.value[i] = constraint(i);
  }  // end of i loop

  return constraintMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::LagrangianMetrics createLagrangianMetricsMsg(
    scalar_t time,
    LagrangianMetricsConstRef metrics) {
  ocs2_msgs::msg::LagrangianMetrics metricsMsg;

  metricsMsg.time = time;
  metricsMsg.penalty = metrics.penalty;

  metricsMsg.constraint.resize(metrics.constraint.size());
  for (size_t i = 0; i < metrics.constraint.size(); i++) {
    metricsMsg.constraint[i] = metrics.constraint(i);
  }  // end of i loop

  return metricsMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::Multiplier createMultiplierMsg(scalar_t time,
                                               MultiplierConstRef multiplier) {
  ocs2_msgs::msg::Multiplier multiplierMsg;

  multiplierMsg.time = time;
  multiplierMsg.penalty = multiplier.penalty;

  multiplierMsg.lagrangian.resize(multiplier.lagrangian.size());
  for (size_t i = 0; i < multiplier.lagrangian.size(); i++) {
    multiplierMsg.lagrangian[i] = multiplier.lagrangian(i);
  }  // end of i loop

  return multiplierMsg;
}

}  // namespace ros_msg_conversions
}  // namespace ocs2
