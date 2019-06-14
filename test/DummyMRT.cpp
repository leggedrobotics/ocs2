/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <string>
#include <iostream>
#include <ros/package.h>
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>
#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

using namespace anymal;
using namespace switched_model;

int main( int argc, char* argv[] )
{

	typedef OCS2AnymalInterface ocs2_robot_interface_t;

	const std::string robotName = "anymal";
	const OCS2AnymalInterface::scalar_t mrtLoopFrequency = 400;

	/******************************************************************************************************/
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFolder = ros::package::getPath("ocs2_anymal_interface") + "/config/" + std::string(argv[1]);
	std::string taskFile = ros::package::getPath("ocs2_anymal_interface") + "/config/" + std::string(argv[1]) + "/task.info";
	std::cerr << "Loading task file: " << taskFile << std::endl;

	ocs2_robot_interface_t::Ptr optimizationInterfacePtr( new ocs2_robot_interface_t(taskFolder) );
	ocs2_robot_interface_t::rbd_state_vector_t initRbdState;
	optimizationInterfacePtr->getLoadedInitialState(initRbdState);

	MRT_ROS_Dummy_Quadruped<12> dummySimulator(
			optimizationInterfacePtr,
			optimizationInterfacePtr->mpcSettings().mrtDesiredFrequency_,
			robotName,
			optimizationInterfacePtr->mpcSettings().mpcDesiredFrequency_);

	dummySimulator.launchNodes(argc, argv);

	// initial state
	MRT_ROS_Dummy_Quadruped<12>::system_observation_t initObservation;
	initObservation.time()  = 0.0;
	optimizationInterfacePtr->computeSwitchedModelState(initRbdState, initObservation.state());
	initObservation.input().setZero();
	initObservation.subsystem() = 15;

	// initial command
	MRT_ROS_Dummy_Quadruped<12>::cost_desired_trajectories_t initCostDesiredTrajectories;
	initCostDesiredTrajectories.desiredTimeTrajectory().resize(1);
	initCostDesiredTrajectories.desiredStateTrajectory().resize(1);
	initCostDesiredTrajectories.desiredInputTrajectory().resize(1);
	MRT_ROS_Dummy_Quadruped<12>::scalar_array_t targetPoseDisplacementVelocity(12, 0.0);
	ocs2::TargetPoseTransformation<MRT_ROS_Dummy_Quadruped<12>::scalar_t>::toCostDesiredState(
			targetPoseDisplacementVelocity, initCostDesiredTrajectories.desiredStateTrajectory().front());

	// run dummy
	dummySimulator.run(initObservation, initCostDesiredTrajectories);

//	xpp_msgs::RobotStateCartesian point;

}
