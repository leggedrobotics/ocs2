#include "ocs2_mpcnet/dummy/MpcnetDummyObserverRos.h"

#include <ros/ros.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MpcnetDummyObserverRos::MpcnetDummyObserverRos(ros::NodeHandle& nodeHandle, std::string topicPrefix) {
  observationPublisher_ = nodeHandle.advertise<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetDummyObserverRos::update(const SystemObservation& observation, const PrimalSolution& primalSolution,
                                    const CommandData& command) {
  auto observationMsg = ros_msg_conversions::createObservationMsg(observation);
  observationPublisher_.publish(observationMsg);
}

}  // namespace ocs2
