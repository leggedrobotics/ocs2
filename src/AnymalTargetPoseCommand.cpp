/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include "ocs2_quadruped_interface/TargetTrajectories_Keyboard_Quadruped.h"

int main( int argc, char* argv[] )
{
	switched_model::TargetTrajectories_Keyboard_Quadruped<double> targetPoseCommand("anymal");

	targetPoseCommand.launchNodes(argc, argv);

	const std::string commadMsg = "Enter XYZ displacement and RollPitchYaw for the robot, separated by spaces";
	targetPoseCommand.getKeyboardCommand(commadMsg);

	// Successful exit
	return 0;
}

