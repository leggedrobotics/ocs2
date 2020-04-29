//
// Created by rgrandia on 04.05.20.
//

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/publisher.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Display.h>

#include <ocs2_msgs/mpc_state.h>

#include "ocs2_anymal_commands/CommandLineInterface.h"

int main(int argc, char* argv[]) {
  std::string filename = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";
  const std::string robotName = "anymal";

  ros::init(argc, argv, robotName + "_mpc_pose_command_node");
  ros::NodeHandle nodeHandle;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  ros::Publisher poseCommandPublisher = nodeHandle.advertise<ocs2_msgs::mpc_state>(robotName + "_mpc_pose_command", 1, true);;

  while (ros::ok() && ros::master::check()) {
    const size_t poseMsgLength = 6;
    ocs2_msgs::mpc_state poseMsg;
    poseMsg.value = std::vector<float>(poseMsgLength, 0.0); // default values

    std::cout <<  "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces: ";

    const auto inputString = getCommandLineString();
    const auto inputWords = stringToWords(inputString);

    try {
      for (int i=0; i< std::min(inputWords.size(), poseMsgLength); ++i){
        poseMsg.value[i] = std::stof(inputWords[i]);
      }

      std::cout << "Published command : " << ocs2::toDelimitedString(poseMsg.value, ", ") << std::endl;

      poseCommandPublisher.publish(poseMsg);
    } catch (...) {
      std::cout <<  "Invalid command : " << inputString << std::endl;
    }
  }

  // Successful exit
  return 0;
}