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
  std::string filename = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const auto targetDisplacementVelocity = pt.get<double>("targetDisplacementVelocity");
  const auto targetRotationVelocity = pt.get<double>("targetRotationVelocity");
  const auto initZHeight = pt.get<double>("comHeight");

  Eigen::Matrix<double, 12, 1> initJoints;
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24> targetPoseCommand(
      argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);

  targetPoseCommand.launchNodes();

  const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
