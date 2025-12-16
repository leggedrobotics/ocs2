/*
 * RosMsgConversions.h
 *
 *  Created on: Jul 7, 2020
 *      Author: Oliver Harley, Marko Bjelonic, Ruben Grandia
 */

#include "ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h"

namespace switched_model {
namespace ros_msg_conversions {

ocs2_switched_model_msgs::msg::Gait toMessage(const Gait& gait) {
  ocs2_switched_model_msgs::msg::Gait msg;
  msg.duration = gait.duration;
  msg.event_phases.reserve(gait.eventPhases.size());
  for (const auto& phase : gait.eventPhases) {
    msg.event_phases.push_back(phase);
  }
  msg.mode_sequence.reserve(gait.modeSequence.size());
  for (const auto& mode : gait.modeSequence) {
    msg.mode_sequence.push_back(mode);
  }
  return msg;
}

Gait fromMessage(const ocs2_switched_model_msgs::msg::Gait& msg) {
  Gait gait;
  gait.duration = msg.duration;
  gait.eventPhases.reserve(msg.event_phases.size());
  for (const auto& phase : msg.event_phases) {
    gait.eventPhases.push_back(phase);
  }
  gait.modeSequence.reserve(msg.mode_sequence.size());
  for (const auto& mode : msg.mode_sequence) {
    gait.modeSequence.push_back(mode);
  }
  assert(isValidGait(gait));
  return gait;
}

ocs2_switched_model_msgs::msg::GaitSequence toMessage(
    const GaitSchedule::GaitSequence& gaitSequence) {
  ocs2_switched_model_msgs::msg::GaitSequence msg;
  msg.gaits.reserve(gaitSequence.size());
  for (const auto& gait : gaitSequence) {
    msg.gaits.emplace_back(toMessage(gait));
  }
  return msg;
}

GaitSchedule::GaitSequence fromMessage(
    const ocs2_switched_model_msgs::msg::GaitSequence& msg) {
  GaitSchedule::GaitSequence gaitSequence;
  gaitSequence.reserve(msg.gaits.size());
  for (const auto& gait : msg.gaits) {
    gaitSequence.emplace_back(fromMessage(gait));
  }
  return gaitSequence;
}

ocs2_switched_model_msgs::msg::ScheduledGaitSequence toMessage(
    scalar_t startTime, const GaitSchedule::GaitSequence& gaitSequence) {
  ocs2_switched_model_msgs::msg::ScheduledGaitSequence msg;
  msg.start_time = startTime;
  msg.gait_sequence = toMessage(gaitSequence);
  return msg;
}

std::pair<scalar_t, GaitSchedule::GaitSequence> fromMessage(
    const ocs2_switched_model_msgs::msg::ScheduledGaitSequence& msg) {
  return {msg.start_time, fromMessage(msg.gait_sequence)};
}

}  // namespace ros_msg_conversions
}  // namespace switched_model
