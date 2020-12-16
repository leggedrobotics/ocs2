/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNodeSqp.h>
#include <ros/init.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"

int main(int argc, char *argv[])
{
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);
  if (programArgs.size() < 3)
  {
    throw std::runtime_error("No robot name and config folder specified. Aborting.");
  }
  const std::string robotName(programArgs[1]);
  const std::string configName(programArgs[2]);

  // Initialize ros node
  ros::init(argc, argv, "anymal_mpc_sqp");
  ros::NodeHandle nodeHandle;

  auto anymalInterface = anymal::getAnymalInterface(anymal::stringToAnymalModel(robotName), anymal::getConfigFolder(configName));
  const auto mpcSettings = ocs2::mpc::loadSettings(anymal::getTaskFilePath(configName));
  ocs2::MultipleShootingSolverSettings sqpSettings;
  sqpSettings.N = 20;
  sqpSettings.n_state = 24;
  sqpSettings.n_input = 24;
  sqpSettings.n_constraint = 12;
  sqpSettings.sqpIteration = 5;
  sqpSettings.constrained = true;
  sqpSettings.qr_decomp = true;
  sqpSettings.printPrimalSol = true;
  sqpSettings.printSolverStatistics = true;
  sqpSettings.printSolverStatus = true;
  sqpSettings.initPrimalSol = false;
  quadrupedMpcNodeSqp(nodeHandle, *anymalInterface, mpcSettings, sqpSettings);

  return 0;
}
