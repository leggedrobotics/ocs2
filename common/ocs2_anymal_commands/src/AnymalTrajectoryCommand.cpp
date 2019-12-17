/******************************************************************************
* File:             AnymalTrajectoryCommand.cpp
*
* Author:           Oliver Harley
* Created:          12/16/19
* Description:      AnymalTrajectoryCommand selects and runs the trajectories made from an external
*                   trajectory optimizer.
*****************************************************************************/


#include <ros/package.h>

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_anymal_commands/Trajectories_Keyboard_Quadruped.h"

int main(int argc, char* argv[]) {
  // std::string filename = ros::package::getPath("ocs2_anymal_commands") + "/config/targetCommand.info";
  // boost::property_tree::ptree pt;
  // boost::property_tree::read_info(filename, pt);

  const auto targetDisplacementVelocity = ("targetDisplacementVelocity");
  const auto targetRotationVelocity = ("targetRotationVelocity");

  const auto initZHeight = ("comHeight");
  Eigen::Matrix<double, 12, 1> initJoints; // "defaultJointState"
  ocs2::loadData::loadEigenMatrix(filename, "defaultJointState", initJoints);

  switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24>
    targetPoseCommand( argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity);

  using quadrupedKeyboard = switched_model::TargetTrajectories_Keyboard_Quadruped<double, 24, 24>;
  auto command_mode = quadrupedKeyboard::COMMAND_MODE::VELOCITY;


  quadrupedKeyboard::scalar_array_t command_limits =
      quadrupedKeyboard::scalar_array_t{2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 40.0, 40.0, 360.0};

  quadrupedKeyboard targetPoseCommand(argc, argv, "anymal", initZHeight, initJoints, targetDisplacementVelocity, targetRotationVelocity,
                                      command_limits, command_mode);

/* ModeSequence Command */
  switched_model::ModeSequence_Keyboard_Quadruped<double> modeSequenceCommand(gaitFile, "anymal", true);
  modeSequenceCommand.launchNodes(argc, argv);
  modeSequenceCommand.getKeyboardCommand();

  /************************************************************************************************
  *                                         launch node                                          *
  ************************************************************************************************/

  trajectoryPoseCommand.launchNodes();

  const std::string commandMsg = "Enter trajectory number:";
  trajectoryPoseCommand.getKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
