/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedDummyNode.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs =
      rclcpp::remove_ros_arguments(argc, argv);

  if (programArgs.size() < 3) {
    throw std::runtime_error(
        "No robot name and config folder specified. Aborting.");
  }
  const std::string urdfPath(programArgs[1]);
  const std::string configName(programArgs[2]);

  // Initialize ros node
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("anymal_mrt");
  std::string urdfString = anymal::getUrdfString(urdfPath);

  auto anymalInterface = anymal::getAnymalInterface(
      urdfString, anymal::getConfigFolder(configName));
  const auto mpcSettings =
      ocs2::mpc::loadSettings(anymal::getTaskFilePath(configName));
  quadrupedDummyNode(node, *anymalInterface, &anymalInterface->getRollout(),
                     mpcSettings.mrtDesiredFrequency_,
                     mpcSettings.mpcDesiredFrequency_);

  return 0;
}
