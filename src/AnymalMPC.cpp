/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <chrono>
#include <string>
#include <iostream>

#include <boost/filesystem.hpp>

#include "ocs2_anymal_interface/MPC_ROS_Anymal.h"

int main( int argc, char* argv[] )
{
	// Time stamp
	std::time_t currentDate = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
	std::cerr << "Current Time: " << std::ctime(&currentDate) << std::endl << std::endl;

	// task file
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	boost::filesystem::path filePath(__FILE__);
	std::string taskFile = filePath.parent_path().parent_path().generic_string() + "/config/" + std::string(argv[1]) + "/task.info";
	std::cout << "Loading task file: " << taskFile << std::endl;

	// launch MPC nodes
	anymal::MPC_ROS_Anymal ocs2AnymalMPC(taskFile);
	ocs2AnymalMPC.launchNodes(argc, argv);
}

