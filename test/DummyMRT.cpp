/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <string>
#include <iostream>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>
#include <pathfile.h>

using namespace anymal;

int main( int argc, char* argv[] )
{

	typedef OCS2AnymalInterface ocs2_robot_interface_t;

	const std::string robotName = "anymal";
	const OCS2AnymalInterface::scalar_t mrtLoopFrequency = 400;

	/******************************************************************************************************/
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string taskFile = std::string(PACKAGE_PATH) + "/config/" + std::string(argv[1]);
	std::cerr << "Loading task file: " << taskFile << std::endl;

	ocs2_robot_interface_t::Ptr optimizationInterfacePtr( new ocs2_robot_interface_t(taskFile) );
	ocs2_robot_interface_t::rbd_state_vector_t initRbdState;
	optimizationInterfacePtr->getLoadedInitialState(initRbdState);

	switched_model::MRT_ROS_Dummy_Quadruped<12> dummySimulator(
			optimizationInterfacePtr,
			optimizationInterfacePtr->mpcSettings().mrtDesiredFrequency_,
			robotName,
			optimizationInterfacePtr->mpcSettings().mpcDesiredFrequency_);

	dummySimulator.launchNodes(argc, argv);

	switched_model::MRT_ROS_Dummy_Quadruped<12>::system_observation_t initObservation;
	initObservation.time()  = 0.0;
	optimizationInterfacePtr->computeSwitchedModelState(initRbdState, initObservation.state());
	initObservation.input().setZero();
	initObservation.subsystem() = 15;
	dummySimulator.init(initObservation);

	dummySimulator.run();

	xpp_msgs::RobotStateCartesian point;

}
