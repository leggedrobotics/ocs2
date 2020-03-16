//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>
#include "ocs2_anymal_wheels_loopshaping/AnymalWheelsLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  const std::string taskName(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic))

  // Initialize ros node
  ros::init(argc, argv, "anymal_wheels_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalWheelsLoopshapingInterface(taskName);
  ocs2::MPC_Settings mpcSettings;
  mpcSettings.loadSettings(anymal::getTaskFilePathAnymalWheelsLoopshaping(taskName));
  ocs2::SLQ_Settings slqSettings;
  slqSettings.loadSettings(anymal::getTaskFilePathAnymalWheelsLoopshaping(taskName));
  quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface, mpcSettings, slqSettings);

  return 0;
}
