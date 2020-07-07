/*
 * RosMsgConversions.h
 *
 *  Created on: Jul 7, 2020
 *      Author: Oliver Harley, Marko Bjelonic, Ruben Grandia
 */

#pragma once

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>

#include "ocs2_switched_model_interface/logic/Gait.h"
#include "ocs2_switched_model_interface/logic/GaitSchedule.h"

#include <switched_model_msgs/gait.h>
#include <switched_model_msgs/gait_sequence.h>
#include <switched_model_msgs/scheduled_gait.h>
#include <switched_model_msgs/trajectory_request.h>

namespace switched_model {
namespace ros_msg_conversions {

template <class ContainerAllocator>
void createGaitMsg(const switched_model::Gait& gait, switched_model_msgs::gait_<ContainerAllocator>& gaitMsg);

template <class ContainerAllocator>
void createGaitSequenceMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence, const std::vector<scalar_t>& startTimes,
                           switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg);

template <class ContainerAllocator>
void createScheduledGaitMsg(const switched_model::Gait& gait, scalar_t startTime,
                            switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg);

template <size_t STATE_DIM, size_t INPUT_DIM>
void createTrajectoryRequestMsg(std::string command, const ocs2::SystemObservation<STATE_DIM, INPUT_DIM>& observation,
                                const scalar_t offsetTime, switched_model_msgs::trajectory_request::Request& requestMsg) {
  ocs2::ros_msg_conversions::createObservationMsg(observation, requestMsg.observation);
  requestMsg.trajectoryCommand = std::move(command);
  requestMsg.offsetTime = offsetTime;
};

void createTrajectoryResponseMsg(const ocs2::CostDesiredTrajectories& costTrajectories,
                                 const switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t>& startTimes,
                                 switched_model_msgs::trajectory_request::Response& responseMsg);

template <class ContainerAllocator>
void readGaitMsg(const switched_model_msgs::gait_<ContainerAllocator>& gaitMsg, switched_model::Gait& gait);

template <class ContainerAllocator>
void readGaitSequenceMsg(const switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
                         switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t>& startTimes);

template <class ContainerAllocator>
void readScheduledGaitMsg(const switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg, scalar_t& startTime,
                          switched_model::Gait& gait);

template <size_t STATE_DIM, size_t INPUT_DIM>
void readTrajectoryRequestMsg(const switched_model_msgs::trajectory_request::Request& requestMsg, std::string& command,
                              ocs2::SystemObservation<STATE_DIM, INPUT_DIM>& observation, scalar_t& offsetTime) {
  ocs2::ros_msg_conversions::readObservationMsg(requestMsg.observation, observation);
  offsetTime = requestMsg.offsetTime;
  command = requestMsg.trajectoryCommand;
};

void readTrajectoryResponseMsg(const switched_model_msgs::trajectory_request::Response& responseMsg,
                               ocs2::CostDesiredTrajectories& costTrajectories, switched_model::GaitSchedule::GaitSequence& gaitSequence,
                               std::vector<scalar_t>& startTimes);

}  // namespace ros_msg_conversions
}  // namespace switched_model

#include "implementation/RosMsgConversions.h"
