/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ocs2_quadruped_interface/TargetTrajectories_Keyboard_Quadruped.h>
#include <ocs2_switched_model_interface/core/Model_Settings.h>

#include <ros/package.h>

int main(int argc, char* argv[]) {
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFile = ros::package::getPath("ocs2_anymal_interface") + "/config/" + std::string(argv[1]) + "/task.info";

  switched_model::Model_Settings modelSettings;
  modelSettings.loadSettings(taskFile);

  Eigen::Matrix<double, 36, 1> initRbdState_;
  ocs2::loadEigenMatrix(taskFile, "initialRobotState", initRbdState_);

  double comHeightOffset = -0.056;  // TODO(Ruben) get from model
  double initZHeight = initRbdState_[5] + comHeightOffset;
  Eigen::Matrix<double, 12, 1> initJoints = initRbdState_.segment(6, 12);

  switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24> targetPoseCommand(
      argc, argv, "anymal", initZHeight, initJoints, modelSettings.targetDisplacementVelocity_, modelSettings.targetRotationVelocity_);

  targetPoseCommand.launchNodes();

  const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
  targetPoseCommand.getKeyboardCommand(commadMsg);

  // Successful exit
  return 0;
}
