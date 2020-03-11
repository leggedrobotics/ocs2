/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

/*
 * namespace switched_model
 * {
 *   extern template void switched_model::quadrupedMpcNode<anymal::WheeledQuadrupedInterface>(ros::NodeHandle& nodeHandle, const
 * anymal::WheeledQuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings, const ocs2::SLQ_Settings& slqSettings);
 * }
 */

/*
 * void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
 *     const ocs2::SLQ_Settings& slqSettings) {
 *   const std::string robotName = "anymal";
 *   using mpc_ros_t = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;
 *
 *   // launch MPC nodes
 *   auto mpcPtr = getMpc(quadrupedInterface, mpcSettings, slqSettings);
 *   mpc_ros_t mpcNode(*mpcPtr, robotName);
 *   mpcNode.launchNodes(nodeHandle);
 * }
 */

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalWheelsInterface(taskName);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  ocs2::SLQ_Settings slqSettings;
  slqSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  switched_model::quadrupedMpcNode<anymal::WheeledQuadrupedInterface>(nodeHandle, *anymalInterface, mpcSettings, slqSettings);

  return 0;
}
