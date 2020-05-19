//
// Created by rgrandia on 04.05.20.
//

#include <ros/package.h>
#include <ros/publisher.h>
#include <ros/ros.h>

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_msgs/mpc_state.h>

#include "ocs2_anymal_commands/CommandLineInterface.h"
#include "ocs2_anymal_commands/PoseCommandToCostDesiredRos.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";

  const std::string filename = [&] {
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    if (programArgs.size() <= 1) {
      throw std::runtime_error("No task file specified. Aborting.");
    }
    return programArgs[1];
  }();

  ros::init(argc, argv, robotName + "_mpc_pose_command_node");
  ros::NodeHandle nodeHandle;

  switched_model::PoseCommandToCostDesiredRos poseCommandPublisher(filename, nodeHandle);
  ros::spinOnce();

  while (ros::ok() && ros::master::check()) {
    std::cout << "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces: ";
    const auto inputString = getCommandLineString();
    const auto inputWords = stringToWords(inputString);

    try {
      switched_model::PoseCommandToCostDesiredRos::PoseCommand_t command{0.0};  // default values
      for (int i = 0; i < std::min(inputWords.size(), command.size()); ++i) {
        command[i] = std::stof(inputWords[i]);
      }

      std::cout << "Published command : " << ocs2::toDelimitedString(command, ", ") << std::endl;

      ros::spinOnce(); // Spin before commanding, to receive latest observation and terrain.
      poseCommandPublisher.publishCostDesiredFromCommand(command);
    } catch (...) {
      std::cout << "Invalid command : " << inputString << std::endl;
    }
  }

  // Successful exit
  return 0;
}