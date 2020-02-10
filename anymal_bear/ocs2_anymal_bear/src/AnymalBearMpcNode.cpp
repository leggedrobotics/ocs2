/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <iostream>
#include <string>

#include <ros/package.h>

#include <ocs2_quadruped_interface/MPC_ROS_Quadruped.h>

#include "ocs2_anymal_bear/AnymalBearInterface.h"

int main(int argc, char* argv[]) {
  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFolder = ros::package::getPath("ocs2_anymal_bear") + "/config/" + std::string(argv[1]);
  std::cerr << "Loading task file from: " << taskFolder << std::endl;

  // Set up interface
  anymal::AnymalBearInterface anymalInterface(taskFolder);

  // launch MPC nodes
  auto mpcPtr = anymalInterface.getMpc();
  switched_model::MPC_ROS_Quadruped<12> ocs2AnymalMPC(*mpcPtr, "anymal");
  ocs2AnymalMPC.launchNodes(argc, argv);
}
