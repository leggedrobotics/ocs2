//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>
#include <ros/init.h>

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_bear_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalBearLoopshapingInterface(anymal::getTaskFileFolderBearLoopshaping(taskName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathBearLoopshaping(taskName));
  const auto ddpSettings = ocs2::ddp::loadSettings(anymal::getTaskFilePathBearLoopshaping(taskName));
  quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface, mpcSettings, ddpSettings);

  return 0;
}
