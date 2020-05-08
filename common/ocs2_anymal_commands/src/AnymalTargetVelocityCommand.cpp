/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_anymal_commands/TargetTrajectories_Keyboard_Quadruped.h"

int main(int argc, char* argv[]) {
  std::string filename;

  if (argc >= 1)
    filename = argv[1];
  else
    filename = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";

  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(filename, pt);
  } catch (boost::property_tree::ptree_bad_path) {
    std::cout << "Could not find/read " << filename << "." << std::endl;
    return 1;  // Fail, and exit.
  }

  const auto targetDisplacementVelocity = pt.get<double>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<double>("targetRotationVelocity");
  const auto initZHeight = pt.get<double>("comHeight");

  Eigen::Matrix<double, 12, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  using quadrupedKeyboard = switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24>;
  auto command_mode = quadrupedKeyboard::COMMAND_MODE::VELOCITY;
  quadrupedKeyboard::scalar_array_t command_limits =
      quadrupedKeyboard::scalar_array_t{2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 40.0, 40.0, 360.0};

  quadrupedKeyboard targetPoseCommand(argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity,
                                      command_limits, command_mode);

  targetPoseCommand.launchNodes();

  const std::string commadMsg = "Enter XYZ Velocity and Angular velocities for the robot, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
