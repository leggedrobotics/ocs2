/*
 * AnymalTargetPoseCommand.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include "ocs2_quadruped_interface/TargetPoseCommand.h"

int main( int argc, char* argv[] )
{
	switched_model::TargetPoseCommand targetPoseCommand("anymal");

	targetPoseCommand.launchNodes(argc, argv);

	targetPoseCommand.run();
}


