/*
 * RosMsgConversions.h
 *
 *  Created on: Jul 7, 2020
 *      Author: Oliver Harley, Marko Bjelonic, Ruben Grandia
 */

#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_switched_model_msgs/msg/gait.hpp>
#include <ocs2_switched_model_msgs/msg/gait_sequence.hpp>
#include <ocs2_switched_model_msgs/msg/scheduled_gait_sequence.hpp>
#include <ocs2_switched_model_msgs/srv/trajectory_request.hpp>

#include "ocs2_switched_model_interface/logic/Gait.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

namespace switched_model {
namespace ros_msg_conversions {

ocs2_switched_model_msgs::msg::Gait toMessage(const Gait& gait);
Gait fromMessage(const ocs2_switched_model_msgs::msg::Gait& msg);

ocs2_switched_model_msgs::msg::GaitSequence toMessage(
    const GaitSchedule::GaitSequence& gaitSequence);
GaitSchedule::GaitSequence fromMessage(
    const ocs2_switched_model_msgs::msg::GaitSequence& msg);

ocs2_switched_model_msgs::msg::ScheduledGaitSequence toMessage(
    scalar_t startTime, const GaitSchedule::GaitSequence& gaitSequence);
std::pair<scalar_t, GaitSchedule::GaitSequence> fromMessage(
    const ocs2_switched_model_msgs::msg::ScheduledGaitSequence& msg);

}  // namespace ros_msg_conversions
}  // namespace switched_model
