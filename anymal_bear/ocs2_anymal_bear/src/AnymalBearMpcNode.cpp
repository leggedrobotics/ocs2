/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <chrono>
#include <iostream>
#include <string>

#include <ros/package.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"
#include "ocs2_anymal_bear/AnymalBearMpcRos.h"

int main(int argc, char* argv[]) {
  // Time stamp
  std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFolder = ros::package::getPath("ocs2_anymal_bear") + "/config/" + std::string(argv[1]);
  std::cerr << "Loading task file: " << taskFolder << std::endl;

  // Set up interface
  anymal::AnymalBearInterface anymalInterface(taskFolder);

  // launch MPC nodes
  anymal::AnymalBearMpcRos ocs2AnymalMPC(anymalInterface.getMpc(), "anymal");
  ocs2AnymalMPC.launchNodes(argc, argv);
}
