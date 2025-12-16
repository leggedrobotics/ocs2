//
// Created by rgrandia on 13.02.20.
//

#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingDummyNode.h>

#include "ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h"
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
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("anymal_loopshaping_mrt");

  std::string urdfString = anymal::getUrdfString(urdfPath);

  auto anymalInterface = anymal::getAnymalLoopshapingInterface(
      urdfString, anymal::getConfigFolderLoopshaping(configName));
  const auto mpcSettings =
      ocs2::mpc::loadSettings(anymal::getTaskFilePathLoopshaping(configName));
  quadrupedLoopshapingDummyNode(node, *anymalInterface,
                                mpcSettings.mrtDesiredFrequency_,
                                mpcSettings.mpcDesiredFrequency_);

  return 0;
}
