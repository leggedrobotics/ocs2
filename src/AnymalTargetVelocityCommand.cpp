/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/TargetTrajectories_Keyboard_Quadruped.h>

int main( int argc, char* argv[] )
{
  using quadrupedKeyboard = switched_model::TargetTrajectories_Keyboard_Quadruped<double>;
  auto command_mode = quadrupedKeyboard::COMMAND_MODE::VELOCITY;
  quadrupedKeyboard::scalar_array_t command_limits = quadrupedKeyboard::scalar_array_t{
    2.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 40.0, 40.0, 360.0};
	switched_model::TargetTrajectories_Keyboard_Quadruped<double> targetPoseCommand("anymal", command_limits, command_mode);

	targetPoseCommand.launchNodes(argc, argv);

	const std::string commadMsg = "Enter XYZ Velocity and Angular velocities for the robot, separated by spaces";
	targetPoseCommand.getKeyboardCommand(commadMsg);

	// Successful exit
	return 0;
}

