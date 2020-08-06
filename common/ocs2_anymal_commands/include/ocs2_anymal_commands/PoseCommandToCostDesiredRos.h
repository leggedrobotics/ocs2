//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <ocs2_core/misc/Lockable.h>

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_state.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model {

class PoseCommandToCostDesiredRos {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t initZHeight;
  joint_coordinate_t defaultJointState;

  using PoseCommand_t = std::array<double, 6>; // [x, y, z, roll, pitch, yaw]

  PoseCommandToCostDesiredRos(const std::string& configFile, ros::NodeHandle& nodeHandle);

  void publishCostDesiredFromCommand(const PoseCommand_t& command);

 private:
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  void terrainCallback(const visualization_msgs::Marker::ConstPtr& msg);

  scalar_t desiredTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const;

  ros::Publisher costDesiredPublisher_;

  ros::Subscriber observationSubscriber_;
  std::mutex observationMutex_;
  ocs2_msgs::mpc_observation::ConstPtr observation_;

  ros::Subscriber terrainSubscriber_;
  std::mutex terrainMutex_;
  TerrainPlane localTerrain_;

  ros::Subscriber commandSubscriber_;
};

}