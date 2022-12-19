//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>
#include <ros/init.h>

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
  ros::init(argc, argv, "anymal_loopshaping_mrt");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  nodeHandle.getParam(descriptionName, urdfString);

  auto anymalInterface = anymal::getAnymalLoopshapingInterface(urdfString, anymal::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathLoopshaping(configName));
  quadrupedLoopshapingDummyNode(nodeHandle, *anymalInterface, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_);

  return 0;
}
