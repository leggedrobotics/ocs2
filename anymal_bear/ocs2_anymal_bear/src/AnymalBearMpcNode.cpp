/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_bear_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalBearInterface(taskName);
  const auto slqSettings = ocs2::loadSlqSettings(anymal::getTaskFilePathBear(taskName));
  const auto mpcSettings = ocs2::loadMpcSettings(anymal::getTaskFilePathBear(taskName));
  quadrupedMpcNode(nodeHandle, *anymalInterface, mpcSettings, slqSettings);

  return 0;
}
