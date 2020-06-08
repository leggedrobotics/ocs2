#pragma once

#include <ocs2_switched_model_interface/logic/Gait.h>
#include <ocs2_switched_model_interface/logic/GaitSchedule.h>
#include <ocs2_switched_model_ros_interfaces/RosMsgConversions.h>
#include <switched_model_msgs/gait.h>
#include <switched_model_msgs/gait_sequence.h>
#include <switched_model_msgs/scheduled_gait.h>
#include <switched_model_msgs/trajectory_request.h>

namespace switched_model {
namespace ros_msg_conversions {

template <class ContainerAllocator>
void createGaitMsg(const switched_model::Gait& gait, switched_model_msgs::gait_<ContainerAllocator>& gaitMsg);

template <class ContainerAllocator>
void createGaitSequenceMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence, const std::vector<scalar_t> startTimes,
                           switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg);

template <class ContainerAllocator>
void createScheduledGaitMsg(const switched_model::Gait& gait, scalar_t startTime,
                            switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg);

template <class ContainerAllocator>
void createTrajectoryRequestMsg(const ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                                const scalar_t offsetTime,
                                switched_model_srvs::trajectory_request::Request<ContainerAllocator>& requestMsg);

template <class ContainerAllocator>
void createTrajectoryResponseMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence,
                                 const ocs2::CostDesiredTrajectories& costTrajectories, std::vector<scalar_t> startTimes,
                                 switched_model_srvs::trajectory_request::Response<ContainerAllocator>& responseMsg);

template <class ContainerAllocator>
void readGaitMsg(const switched_model_msgs::gait_<ContainerAllocator>& gaitMsg, switched_model::Gait& gait);

template <class ContainerAllocator>
void readGaitSequenceMsg(const switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
                         switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t> startTimes);

template <class ContainerAllocator>
void readScheduledGaitMsg(const switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg, scalar_t& startTime,
                          switched_model::Gait& gait);

template <class ContainerAllocator>
void readTrajectoryRequestMsg(const switched_model_srvs::trajectory_request::Request<ContainerAllocator>& requestMsg,
                              ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                              scalar_t& offsetTime);

template <class ContainerAllocator>
void readTrajectoryResponseMsg(const switched_model_srvs::trajectory_request::Response<ContainerAllocator>& responseMsg,
                               CostDesiredTrajectories& costTrajectories, switched_model::GaitSchedule::GaitSequence& gaitSequence,
                               std::vector<scalar_t> startTimes);

}  // namespace ros_msg_conversions
}  // namespace switched_model

#include "implementation/RosMsgConversions.h"
