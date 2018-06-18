/*
 * DummyMRT.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"
#include <ocs2_quadruped_interface/test/MRT_ROS_Dummy_Quadruped.h>

using namespace anymal;

int main( int argc, char* argv[] )
{

	typedef OCS2AnymalInterface ocs2_robot_interface_t;

	const std::string robotName = "anymal";
	const OCS2AnymalInterface::scalar_t mrtLoopFrequency = 250;

	/******************************************************************************************************/
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	boost::filesystem::path filePath(__FILE__);
	std::string taskFile = filePath.parent_path().parent_path().generic_string() + "/config/" + std::string(argv[1]) + "/task.info";
	std::cout << "Loading task file: " << taskFile << std::endl;

	ocs2_robot_interface_t::Ptr optimizationInterfacePtr( new ocs2_robot_interface_t(taskFile) );
	ocs2_robot_interface_t::rbd_state_vector_t initRbdState;
	optimizationInterfacePtr->getLoadedInitialState(initRbdState);

	switched_model::MRT_ROS_Dummy_Quadruped<12> dummySimulator(
			optimizationInterfacePtr, mrtLoopFrequency, robotName);

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
