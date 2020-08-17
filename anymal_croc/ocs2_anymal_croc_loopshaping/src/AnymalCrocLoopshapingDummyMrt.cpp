//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>
#include <ros/init.h>

#include "ocs2_anymal_croc_loopshaping/AnymalCrocLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_croc_loopshaping_mrt");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalCrocLoopshapingInterface(anymal::getTaskFileFolderCrocLoopshaping(taskName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathCrocLoopshaping(taskName));
  quadrupedLoopshapingDummyNode(nodeHandle, *anymalInterface, mpcSettings.mrtDesiredFrequency_, mpcSettings.mpcDesiredFrequency_);

  return 0;
}
