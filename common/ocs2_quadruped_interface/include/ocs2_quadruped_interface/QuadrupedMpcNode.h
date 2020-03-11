//
// Created by rgrandia on 17.02.20.
//

#pragma once

#include <ros/node_handle.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ddp/SLQ_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

template <class QuadInterface>
void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
                      const ocs2::SLQ_Settings& slqSettings) {
  const std::string robotName = "anymal";
  using mpc_ros_t = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;

  // launch MPC nodes
  auto mpcPtr = getMpc<QuadInterface>(quadrupedInterface, mpcSettings, slqSettings);
  mpc_ros_t mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}

}  // namespace switched_model
