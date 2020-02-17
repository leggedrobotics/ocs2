//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>

#include "ocs2_anymal_bear_loopshaping/AnymalBearLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_bear_loopshaping_mrt");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalBearLoopshapingInterface(taskName);
  quadrupedLoopshapingDummyNode(nodeHandle, *anymalInterface);

  return 0;
}