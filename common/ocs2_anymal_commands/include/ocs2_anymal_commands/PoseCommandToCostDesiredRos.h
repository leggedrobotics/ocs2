//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model {

class PoseCommandToCostDesiredRos {
 public:
  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t comHeight;
  joint_coordinate_t defaultJointState;

  PoseCommandToCostDesiredRos(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix, const scalar_array_t& targetCommandLimits,
                              const std::string& configFile);

  void publishKeyboardCommand(const std::string& commadMsg) { targetTrajectoriesKeyboardPublisherPtr_->publishKeyboardCommand(commadMsg); }

 private:
  scalar_t desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const;
  ocs2::TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const ocs2::SystemObservation& observation) const;

  void terrainCallback(const visualization_msgs::Marker::ConstPtr& msg);

  std::unique_ptr<ocs2::TargetTrajectoriesKeyboardPublisher> targetTrajectoriesKeyboardPublisherPtr_;

  TerrainPlane localTerrain_;
  mutable std::mutex terrainMutex_;
  ros::Subscriber terrainSubscriber_;
};

}
