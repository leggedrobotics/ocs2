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

#include <ocs2_switched_model_msgs/gait.h>
#include <ocs2_switched_model_msgs/gait_sequence.h>
#include <ocs2_switched_model_msgs/scheduled_gait_sequence.h>
#include <ocs2_switched_model_msgs/trajectory_request.h>

#include "ocs2_switched_model_interface/logic/Gait.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

namespace switched_model {
namespace ros_msg_conversions {

ocs2_switched_model_msgs::gait toMessage(const Gait& gait);
Gait fromMessage(const ocs2_switched_model_msgs::gait& msg);

ocs2_switched_model_msgs::gait_sequence toMessage(const GaitSchedule::GaitSequence& gaitSequence);
GaitSchedule::GaitSequence fromMessage(const ocs2_switched_model_msgs::gait_sequence& msg);

ocs2_switched_model_msgs::scheduled_gait_sequence toMessage(scalar_t startTime, const GaitSchedule::GaitSequence& gaitSequence);
std::pair<scalar_t, GaitSchedule::GaitSequence> fromMessage(const ocs2_switched_model_msgs::scheduled_gait_sequence& msg);

}  // namespace ros_msg_conversions
}  // namespace switched_model
