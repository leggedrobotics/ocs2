/*
 * RosMsgConversions.h
 *
 *  Created on: Jul 7, 2020
 *      Author: Oliver Harley, Marko Bjelonic, Ruben Grandia
 */

#include "ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h"

namespace switched_model {
namespace ros_msg_conversions {

ocs2_switched_model_msgs::gait toMessage(const Gait& gait) {
  ocs2_switched_model_msgs::gait msg;
  msg.duration = gait.duration;
  msg.eventPhases.reserve(gait.eventPhases.size());
  for (const auto& phase : gait.eventPhases) {
    msg.eventPhases.push_back(phase);
  }
  msg.modeSequence.reserve(gait.modeSequence.size());
  for (const auto& mode : gait.modeSequence) {
    msg.modeSequence.push_back(mode);
  }
  return msg;
}

Gait fromMessage(const ocs2_switched_model_msgs::gait& msg) {
  Gait gait;
  gait.duration = msg.duration;
  gait.eventPhases.reserve(msg.eventPhases.size());
  for (const auto& phase : msg.eventPhases) {
    gait.eventPhases.push_back(phase);
  }
  gait.modeSequence.reserve(msg.modeSequence.size());
  for (const auto& mode : msg.modeSequence) {
    gait.modeSequence.push_back(mode);
  }
  assert(gait.eventPhase.size() + 1 == gait.modeSequence.size());
  return gait;
}

ocs2_switched_model_msgs::gait_sequence toMessage(const GaitSchedule::GaitSequence& gaitSequence) {
  ocs2_switched_model_msgs::gait_sequence msg;
  msg.gaits.reserve(gaitSequence.size());
  for (const auto& gait : gaitSequence) {
    msg.gaits.emplace_back(toMessage(gait));
  }
  return msg;
}

GaitSchedule::GaitSequence fromMessage(const ocs2_switched_model_msgs::gait_sequence& msg) {
  GaitSchedule::GaitSequence gaitSequence;
  gaitSequence.reserve(msg.gaits.size());
  for (const auto& gait : msg.gaits) {
    gaitSequence.emplace_back(fromMessage(gait));
  }
  return gaitSequence;
}

ocs2_switched_model_msgs::scheduled_gait_sequence toMessage(scalar_t startTime, const GaitSchedule::GaitSequence& gaitSequence) {
  ocs2_switched_model_msgs::scheduled_gait_sequence msg;
  msg.startTime = startTime;
  msg.gaitSequence = toMessage(gaitSequence);
  return msg;
}

std::pair<scalar_t, GaitSchedule::GaitSequence> fromMessage(const ocs2_switched_model_msgs::scheduled_gait_sequence& msg) {
  return {msg.startTime, fromMessage(msg.gaitSequence)};
}

ocs2_switched_model_msgs::trajectory_request::Request toTrajectoryRequest(std::string command, const ocs2::SystemObservation& observation,
                                                                          scalar_t offsetTime) {
  ocs2_switched_model_msgs::trajectory_request::Request request;
  request.offsetTime = offsetTime;
  request.trajectoryCommand = std::move(command);
  ocs2::ros_msg_conversions::createObservationMsg(observation, request.observation);
  return request;
}

std::tuple<std::string, ocs2::SystemObservation, scalar_t> fromTrajectoryRequest(
    const ocs2_switched_model_msgs::trajectory_request::Request& request) {
  ocs2::SystemObservation observation;
  ocs2::ros_msg_conversions::readObservationMsg(request.observation, observation);
  return {request.trajectoryCommand, std::move(observation), request.offsetTime};
}

ocs2_switched_model_msgs::trajectory_request::Response toTrajectoryResponse(const ocs2::CostDesiredTrajectories& costTrajectories,
                                                                            const GaitSchedule::GaitSequence& gaitSequence) {
  ocs2_switched_model_msgs::trajectory_request::Response response;
  ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(costTrajectories, response.trajectory);
  response.gaitSequence = toMessage(gaitSequence);
  return response;
}

std::pair<ocs2::CostDesiredTrajectories, GaitSchedule::GaitSequence> fromTrajectoryResponse(
    const ocs2_switched_model_msgs::trajectory_request::Response& response) {
  ocs2::CostDesiredTrajectories costTrajectories;
  ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(response.trajectory, costTrajectories);
  return {std::move(costTrajectories), fromMessage(response.gaitSequence)};
}

}  // namespace ros_msg_conversions
}  // namespace switched_model
