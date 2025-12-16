//
// Created by rgrandia on 10.03.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>

#include "rclcpp/rclcpp.hpp"

namespace switched_model {

class QuadrupedTfPublisher {
 public:
  QuadrupedTfPublisher() = default;

  void launchNode(const rclcpp::Node::SharedPtr& node,
                  const std::string& descriptionName,
                  std::vector<std::string> jointNames, std::string baseName,
                  const std::string& tfPrefix = "");

  void publish(rclcpp::Time timeStamp, const vector_t& state,
               const std::string& worldFrame);

  void publish(rclcpp::Time timeStamp, const base_coordinate_t& basePose,
               const joint_coordinate_t& jointPositions,
               const std::string& worldFrame);

 private:
  void updateJointPositions(const joint_coordinate_t& jointPositions);
  void updateBasePose(rclcpp::Time timeStamp, const base_coordinate_t& basePose,
                      const std::string& worldFrame);

  rclcpp::Node::SharedPtr node_;
  // Publishers
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;

  // Messages
  std::string tfPrefix_ = "";
  std::string baseName_;
  std::vector<std::string> jointNames_;
  std::map<std::string, double> jointPositionsMap_;
  geometry_msgs::msg::TransformStamped baseToWorldTransform_;
  rclcpp::Time lastTimeStamp_;
};

}  // namespace switched_model