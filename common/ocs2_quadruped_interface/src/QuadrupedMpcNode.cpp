//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedMpcNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <ocs2_switched_model_interface/logic/GaitReceiver.h>

#include <ocs2_quadruped_interface/QuadrupedSlqMpc.h>

namespace switched_model {

void quadrupedMpcNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::MPC_Settings& mpcSettings,
                      const ocs2::SLQ_Settings& slqSettings) {
  const std::string robotName = "anymal";
  using mpc_ros_t = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;

  auto gaitReceiver = std::make_shared<switched_model::GaitReceiver>(nodeHandle, quadrupedInterface.getLogicRulesPtr(), robotName);

  auto solverModules = quadrupedInterface.getSynchronizedModules();
  solverModules.insert(solverModules.begin(), gaitReceiver);

  // Prepare Mpc object
  auto mpcPtr = getMpc(quadrupedInterface, mpcSettings, slqSettings);
  mpcPtr->getSolverPtr()->setSynchronizedModules(solverModules);

  // launch MPC node
  mpc_ros_t mpcNode(*mpcPtr, robotName);
  mpcNode.launchNodes(nodeHandle);
}

}  // namespace switched_model
