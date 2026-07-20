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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace ocs2 {
namespace ros_msg_conversions {
namespace {

template <typename Scalar>
void requireFinite(Scalar value, const std::string& field) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("[RosMsgConversions] " + field + " must be finite.");
  }
}

template <typename Container>
void requireFiniteValues(const Container& values, const std::string& field) {
  for (size_t i = 0; i < values.size(); ++i) {
    requireFinite(values[i], field + "[" + std::to_string(i) + "]");
  }
}

float checkedFloat(scalar_t value, const std::string& field) {
  requireFinite(value, field);
  if (std::abs(value) > static_cast<scalar_t>(std::numeric_limits<float>::max())) {
    throw std::runtime_error("[RosMsgConversions] " + field + " exceeds the float32 message range.");
  }
  return static_cast<float>(value);
}

void validateModeSchedule(const scalar_array_t& eventTimes, const size_array_t& modeSequence) {
  if (modeSequence.size() != eventTimes.size() + 1) {
    throw std::runtime_error("[RosMsgConversions] mode_sequence size must equal event_times size plus one.");
  }
  for (size_t i = 0; i < eventTimes.size(); ++i) {
    requireFinite(eventTimes[i], "mode_schedule.event_times[" + std::to_string(i) + "]");
    if (i > 0 && eventTimes[i] < eventTimes[i - 1]) {
      throw std::runtime_error("[RosMsgConversions] mode_schedule.event_times must be non-decreasing.");
    }
  }
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcObservation createObservationMsg(
    const SystemObservation& observation) {
  ocs2_msgs::msg::MpcObservation observationMsg;

  requireFinite(observation.time, "observation.time");
  if (observation.state.size() == 0) {
    throw std::runtime_error("[RosMsgConversions] observation.state must not be empty.");
  }
  observationMsg.time = observation.time;

  observationMsg.state.value.resize(observation.state.rows());
  for (size_t i = 0; i < observation.state.rows(); i++) {
    observationMsg.state.value[i] = checkedFloat(observation.state(i), "observation.state[" + std::to_string(i) + "]");
  }

  observationMsg.input.value.resize(observation.input.rows());
  for (size_t i = 0; i < observation.input.rows(); i++) {
    observationMsg.input.value[i] = checkedFloat(observation.input(i), "observation.input[" + std::to_string(i) + "]");
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

  requireFinite(observationMsg.time, "observation.time");
  if (observationMsg.state.value.empty()) {
    throw std::runtime_error("[RosMsgConversions] observation.state must not be empty.");
  }
  requireFiniteValues(observationMsg.state.value, "observation.state");
  requireFiniteValues(observationMsg.input.value, "observation.input");
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
  validateModeSchedule(modeSchedule.eventTimes, modeSchedule.modeSequence);
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

  validateModeSchedule(eventTimes, mode_sequence);

  return {eventTimes, mode_sequence};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ocs2_msgs::msg::MpcPerformanceIndices createPerformanceIndicesMsg(
    scalar_t initTime,
    const PerformanceIndex& performanceIndices) {
  ocs2_msgs::msg::MpcPerformanceIndices performanceIndicesMsg;

  performanceIndicesMsg.init_time = checkedFloat(initTime, "performance_indices.init_time");
  performanceIndicesMsg.merit = checkedFloat(performanceIndices.merit, "performance_indices.merit");
  performanceIndicesMsg.cost = checkedFloat(performanceIndices.cost, "performance_indices.cost");
  performanceIndicesMsg.dynamics_violation_sse =
      checkedFloat(performanceIndices.dynamicsViolationSSE, "performance_indices.dynamics_violation_sse");
  performanceIndicesMsg.equality_constraints_sse =
      checkedFloat(performanceIndices.equalityConstraintsSSE, "performance_indices.equality_constraints_sse");
  performanceIndicesMsg.equality_lagrangian =
      checkedFloat(performanceIndices.equalityLagrangian, "performance_indices.equality_lagrangian");
  performanceIndicesMsg.inequality_lagrangian =
      checkedFloat(performanceIndices.inequalityLagrangian, "performance_indices.inequality_lagrangian");

  return performanceIndicesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex readPerformanceIndicesMsg(
    const ocs2_msgs::msg::MpcPerformanceIndices& performanceIndicesMsg) {
  requireFinite(performanceIndicesMsg.init_time, "performance_indices.init_time");
  requireFinite(performanceIndicesMsg.merit, "performance_indices.merit");
  requireFinite(performanceIndicesMsg.cost, "performance_indices.cost");
  requireFinite(performanceIndicesMsg.dynamics_violation_sse, "performance_indices.dynamics_violation_sse");
  requireFinite(performanceIndicesMsg.equality_constraints_sse, "performance_indices.equality_constraints_sse");
  requireFinite(performanceIndicesMsg.equality_lagrangian, "performance_indices.equality_lagrangian");
  requireFinite(performanceIndicesMsg.inequality_lagrangian, "performance_indices.inequality_lagrangian");
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
  const size_t N = stateTrajectory.size();
  if (N == 0) {
    throw std::runtime_error("[RosMsgConversions] target state trajectory must not be empty.");
  }
  if (timeTrajectory.size() != N) {
    throw std::runtime_error("[RosMsgConversions] target time/state trajectory lengths do not match.");
  }
  if (!inputTrajectory.empty() && inputTrajectory.size() != N) {
    throw std::runtime_error("[RosMsgConversions] target input/state trajectory lengths do not match.");
  }

  const auto stateDim = stateTrajectory.front().size();
  if (stateDim == 0) {
    throw std::runtime_error("[RosMsgConversions] target state vectors must not be empty.");
  }
  const auto inputDim = inputTrajectory.empty() ? 0 : inputTrajectory.front().size();
  for (size_t i = 0; i < N; ++i) {
    requireFinite(timeTrajectory[i], "target.time_trajectory[" + std::to_string(i) + "]");
    if (i > 0 && timeTrajectory[i] < timeTrajectory[i - 1]) {
      throw std::runtime_error("[RosMsgConversions] target time trajectory must be non-decreasing.");
    }
    if (stateTrajectory[i].size() != stateDim || !stateTrajectory[i].allFinite()) {
      throw std::runtime_error("[RosMsgConversions] target state trajectory has inconsistent dimensions or non-finite values.");
    }
    if (!inputTrajectory.empty() && (inputTrajectory[i].size() != inputDim || !inputTrajectory[i].allFinite())) {
      throw std::runtime_error("[RosMsgConversions] target input trajectory has inconsistent dimensions or non-finite values.");
    }
  }

  // time and state
  targetTrajectoriesMsg.time_trajectory.resize(N);
  targetTrajectoriesMsg.state_trajectory.resize(N);
  for (size_t i = 0; i < N; i++) {
    targetTrajectoriesMsg.time_trajectory[i] = timeTrajectory[i];
    auto& values = targetTrajectoriesMsg.state_trajectory[i].value;
    values.resize(stateDim);
    for (size_t j = 0; j < stateDim; ++j) {
      values[j] = checkedFloat(stateTrajectory[i](j), "target.state_trajectory");
    }
  }  // end of i loop

  // input
  targetTrajectoriesMsg.input_trajectory.resize(inputTrajectory.size());
  for (size_t i = 0; i < inputTrajectory.size(); i++) {
    auto& values = targetTrajectoriesMsg.input_trajectory[i].value;
    values.resize(inputDim);
    for (size_t j = 0; j < inputDim; ++j) {
      values[j] = checkedFloat(inputTrajectory[i](j), "target.input_trajectory");
    }
  }  // end of i loop

  return targetTrajectoriesMsg;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories readTargetTrajectoriesMsg(
    const ocs2_msgs::msg::MpcTargetTrajectories& targetTrajectoriesMsg) {
  const size_t stateTrajectorySize = targetTrajectoriesMsg.state_trajectory.size();
  const size_t timeTrajectorySize = targetTrajectoriesMsg.time_trajectory.size();
  const size_t inputTrajectorySize = targetTrajectoriesMsg.input_trajectory.size();
  if (stateTrajectorySize == 0) {
    throw std::runtime_error(
        "An empty target trajectories message is received.");
  }
  if (timeTrajectorySize != stateTrajectorySize) {
    throw std::runtime_error(
        "Target trajectories message has mismatched time/state trajectory lengths.");
  }
  if (inputTrajectorySize != 0 && inputTrajectorySize != stateTrajectorySize) {
    throw std::runtime_error(
        "Target trajectories message has mismatched input/state trajectory lengths.");
  }

  // state and time
  const size_t N = stateTrajectorySize;
  scalar_array_t desiredTimeTrajectory(N);
  vector_array_t desiredStateTrajectory(N);
  const size_t stateDim = targetTrajectoriesMsg.state_trajectory.front().value.size();
  if (stateDim == 0) {
    throw std::runtime_error("Target trajectories message has empty state vectors.");
  }
  for (size_t i = 0; i < N; i++) {
    desiredTimeTrajectory[i] = targetTrajectoriesMsg.time_trajectory[i];
    requireFinite(desiredTimeTrajectory[i], "target.time_trajectory[" + std::to_string(i) + "]");
    if (i > 0 && desiredTimeTrajectory[i] < desiredTimeTrajectory[i - 1]) {
      throw std::runtime_error(
          "Target trajectories message has decreasing time trajectory.");
    }
    if (targetTrajectoriesMsg.state_trajectory[i].value.size() != stateDim) {
      throw std::runtime_error("Target trajectories message has inconsistent state dimensions.");
    }
    requireFiniteValues(targetTrajectoriesMsg.state_trajectory[i].value, "target.state_trajectory");

    desiredStateTrajectory[i] =
        Eigen::Map<const Eigen::VectorXf>(
            targetTrajectoriesMsg.state_trajectory[i].value.data(),
            targetTrajectoriesMsg.state_trajectory[i].value.size())
            .cast<scalar_t>();
  }  // end of i loop

  // input
  vector_array_t desiredInputTrajectory(inputTrajectorySize);
  const size_t inputDim = inputTrajectorySize == 0 ? 0 : targetTrajectoriesMsg.input_trajectory.front().value.size();
  for (size_t i = 0; i < inputTrajectorySize; i++) {
    if (targetTrajectoriesMsg.input_trajectory[i].value.size() != inputDim) {
      throw std::runtime_error("Target trajectories message has inconsistent input dimensions.");
    }
    requireFiniteValues(targetTrajectoriesMsg.input_trajectory[i].value, "target.input_trajectory");
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
