
#include <ros/package.h>

#include "ocs2_anymal_commands/AnimatronicCommandToCostDesiredRos.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";
  std::string motionFile = ros::package::getPath("ocs2_anymal_commands") + "/config/motions.info";
  std::cerr << "Loading motion file: " << motionFile << std::endl;

  ros::init(argc, argv, robotName + "_mpc_motion_command");
  ros::NodeHandle nodeHandle;

  switched_model::AnimatronicCommandToCostDesiredRos commandInterface(nodeHandle, motionFile, robotName);

  ros::Rate rate(10);
  while (ros::ok() && ros::master::check()) {
    commandInterface.getKeyboardCommand();
    rate.sleep();
  }

  // Successful exit
  return 0;
}
