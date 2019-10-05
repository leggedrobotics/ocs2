/*
 * AnymalModeSequenceCommand.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#include <ros/package.h>

#include "ocs2_anymal_commands/ModeSequence_Keyboard_Quadruped.h"

int main(int argc, char* argv[]) {
  std::string gaitFile = ros::package::getPath("ocs2_anymal_commands") + "/config/gait.info";
  std::cerr << "Loading gait file: " << gaitFile << std::endl;

  switched_model::ModeSequence_Keyboard_Quadruped<double> modeSequenceCommand(gaitFile, "anymal", true);

  modeSequenceCommand.launchNodes(argc, argv);

  modeSequenceCommand.getKeyboardCommand();

  // Successful exit
  return 0;
}
