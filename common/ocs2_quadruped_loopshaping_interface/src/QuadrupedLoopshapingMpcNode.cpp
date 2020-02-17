//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

namespace switched_model {

void quadrupedLoopshapingMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedLoopshapingInterface& quadrupedInterface) {
  static constexpr size_t STATE_DIM = 48;
  static constexpr size_t INPUT_DIM = 24;
  const std::string robotName = "anymal";
  using mpc_ros_t = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;

  // launch MPC nodes
  auto mpcPtr = quadrupedInterface.getMpc();
  mpc_ros_t mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}

}  // namespace switched_model
