//
// Created by rgrandia on 10.03.22.
//

#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

class QuadrupedTfPublisher {
 public:
  QuadrupedTfPublisher() = default;

  void launchNode(ros::NodeHandle& nodeHandle, const std::string& descriptionName, std::vector<std::string> jointNames,
                  std::string baseName, const std::string& tfPrefix = "");

  void publish(ros::Time timeStamp, const vector_t& state, const std::string& worldFrame);

  void publish(ros::Time timeStamp, const base_coordinate_t& basePose, const joint_coordinate_t& jointPositions,
               const std::string& worldFrame);

 private:
  void updateJointPositions(const joint_coordinate_t& jointPositions);
  void updateBasePose(ros::Time timeStamp, const base_coordinate_t& basePose, const std::string& worldFrame);

  // Publishers
  tf::TransformBroadcaster tfBroadcaster_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  // Messages
  std::string tfPrefix_ = "";
  std::string baseName_;
  std::vector<std::string> jointNames_;
  std::map<std::string, double> jointPositionsMap_;
  geometry_msgs::TransformStamped baseToWorldTransform_;
  ros::Time lastTimeStamp_ = ros::Time::now();
};

}  // namespace switched_model