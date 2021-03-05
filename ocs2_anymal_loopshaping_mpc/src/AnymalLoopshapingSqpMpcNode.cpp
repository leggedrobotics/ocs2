//
// Created by rgrandia on 13.02.20.
//

#include <ros/init.h>

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpcNode.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingSqpMpc.h>

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_loopshaping_mpc");
  ros::NodeHandle nodeHandle;

  auto anymalInterface =
      anymal::getAnymalLoopshapingInterface(anymal::stringToAnymalModel(robotName), anymal::getConfigFolderLoopshaping(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePathLoopshaping(configName));
  ocs2::MultipleShootingSolverSettings sqpSettings;
  sqpSettings.dt = 0.02;  // High resolution needed due to loopshaping!
  sqpSettings.n_state = 48;
  sqpSettings.n_input = 24;
  sqpSettings.sqpIteration = 1;
  sqpSettings.deltaTol = 1e-2;
  sqpSettings.inequalityConstraintMu = 0.1;
  sqpSettings.inequalityConstraintDelta = 5.0;
  sqpSettings.qr_decomp = true;
  sqpSettings.printSolverStatistics = true;
  sqpSettings.printSolverStatus = false;
  sqpSettings.printModeScheduleDebug = false;
  sqpSettings.printLinesearch = false;

  auto mpcPtr = getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
  quadrupedLoopshapingMpcNode(nodeHandle, *anymalInterface, std::move(mpcPtr));

  return 0;
}
