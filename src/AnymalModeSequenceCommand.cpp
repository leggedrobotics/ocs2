/*
 * AnymalModeSequenceCommand.cpp
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#include <pathfile.h>
#include "ocs2_quadruped_interface/ModeSequence_Keyboard_Quadruped.h"

int main( int argc, char* argv[] )
{
	// task file
	if ( argc <= 1) throw std::runtime_error("No task file specified. Aborting.");
	std::string gaitFile = std::string(PACKAGE_PATH) + "/config/" + std::string(argv[1]) + "/gait.info";
	std::cerr << "Loading gait file: " << gaitFile << std::endl;

	switched_model::ModeSequence_Keyboard_Quadruped<double> modeSequenceCommand(
			gaitFile, "anymal", true);

	modeSequenceCommand.launchNodes(argc, argv);

	modeSequenceCommand.getKeyboardCommand();

	// Successful exit
	return 0;
}


