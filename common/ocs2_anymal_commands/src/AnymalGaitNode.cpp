/*
 * AnymalModeSequenceCommand.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#include <ros/package.h>

#include "ocs2_anymal_commands/ModeSequenceKeyboard.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";
  std::string gaitFile = ros::package::getPath("ocs2_anymal_commands") + "/config/gait.info";
  std::cerr << "Loading gait file: " << gaitFile << std::endl;

  ros::init(argc, argv, robotName + "_mpc_mode_sequence");
  ros::NodeHandle nodeHandle;

  switched_model::ModeSequenceKeyboard modeSequenceCommand(nodeHandle, gaitFile, robotName, true);

  while (ros::ok() && ros::master::check()) {
    modeSequenceCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
