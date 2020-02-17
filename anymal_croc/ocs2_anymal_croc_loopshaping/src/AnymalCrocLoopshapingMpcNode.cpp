//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>

#include "ocs2_anymal_croc_loopshaping/AnymalCrocLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_croc_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalCrocLoopshapingInterface(taskName);
  quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface);

  return 0;
}
