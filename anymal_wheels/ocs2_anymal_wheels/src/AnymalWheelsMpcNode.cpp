/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>
#include "ocs2_anymal_wheels/AnymalWheelsInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(programArgs[1]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalWheelsInterface(anymal::getTaskFileFolderWheels(taskName));
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  ocs2::SLQ_Settings slqSettings;
  slqSettings.loadSettings(anymal::getTaskFilePathWheels(taskName));
  quadrupedMpcNode(nodeHandle, *anymalInterface, mpcSettings, slqSettings);

  return 0;
}
