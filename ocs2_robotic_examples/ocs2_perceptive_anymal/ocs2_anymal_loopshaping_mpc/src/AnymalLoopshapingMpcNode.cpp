//
// Created by rgrandia on 13.02.20.
//

#include <ros/init.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string descriptionName(programArgs[1]);
  const std::string configName(programArgs[2]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto anymalInterface = anymal::getAnymalLoopshapingInterface(urdfString, anymal::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathLoopshaping(configName));

  switch (anymalInterface->modelSettings().algorithm_) {
    case switched_model::Algorithm::DDP: {
      const auto ddpSettings = ocs2::ddp::loadSettings(anymal::getTaskFilePathLoopshaping(configName));
      auto mpcPtr = getDdpMpc(*anymalInterface, mpcSettings, ddpSettings);
      quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface, std::move(mpcPtr));
      break;
    }
    case switched_model::Algorithm::SQP: {
      const auto sqpSettings = ocs2::sqp::loadSettings(anymal::getConfigFolderLoopshaping(configName) + "/multiple_shooting.info");
      auto mpcPtr = getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
      quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface, std::move(mpcPtr));
      break;
    }
  }

  return 0;
}
