//
// Created by rgrandia on 04.05.20.
//

#pragma once

#include <ros/ros.h>

#include <ocs2_core/misc/Lockable.h>

#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_state.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/terrain/TerrainModel.h>

namespace switched_model {

class PoseCommandToCostDesiredRos {
 public:

  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
  scalar_t initZHeight;
  Eigen::Matrix<scalar_t, 12, 1> defaultJointState;

  PoseCommandToCostDesiredRos(ros::NodeHandle& nodeHandle, ocs2::SharedLockablePPtr<TerrainModel> terrainPptr);

  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  void commandCallback(const ocs2_msgs::mpc_state::ConstPtr& msg);

 private:
  scalar_t estimeTimeToTarget(scalar_t dyaw, scalar_t dx, scalar_t dy) const;

  ocs2::SharedLockablePPtr<TerrainModel> terrainPptr_;
  ros::Publisher costDesiredPublisher_;

  ros::Subscriber observationSubscriber_;
  std::mutex observationMutex_;
  ocs2_msgs::mpc_observation::ConstPtr observation_;

  ros::Subscriber commandSubscriber_;
};

}