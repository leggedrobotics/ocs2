/*
 * LoadTask.h
 *
 *  Created on: Jul 27, 2016
 *      Author: farbod
 */

#ifndef LOADTASK_H_
#define LOADTASK_H_

#include <vector>
#include <string>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_anymal_switched_model/AnymalSwitchedModel.h>


namespace anymal {

void loadModelSettings(const std::string& filename, Options& options,
		double& zmpWeight, double& impulseWeight, double& impulseSigmeFactor,
		bool verbose = true) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	std::cerr<<"==========================================" << std::endl;

	try {
		options.constrainedIntegration_ = pt.get<bool>("model_settings.constrainedIntegration");
		std::cerr << "#### constrainedIntegration ....... " << options.constrainedIntegration_  << std::endl;
	}
	catch (const std::exception& e){
		options.constrainedIntegration_ = true;
		std::cerr << "#### constrainedIntegration ....... " << options.constrainedIntegration_  << "\t(default)" << std::endl;
	}

	try {
		options.useCartesianContactForce_ = pt.get<bool>("model_settings.useCartesianContactForce");
		std::cerr << "#### useCartesianContactForce ..... " << options.useCartesianContactForce_  << std::endl;
	}
	catch (const std::exception& e){
		options.useCartesianContactForce_ = false;
		std::cerr << "#### useCartesianContactForce ..... " << options.useCartesianContactForce_  << "\t(default)" << std::endl;
	}

	try {
		options.contactForceWeight_ = pt.get<double>("model_settings.contactForceWeight");
		std::cerr << "#### contactForceWeight ........... " << options.contactForceWeight_ << std::endl;
	}
	catch (const std::exception& e){
		options.contactForceWeight_ = 0.1;
		std::cerr << "#### contactForceWeight ........... " << options.contactForceWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.zDirectionPositionWeight_ = pt.get<double>("model_settings.zDirectionPositionWeight");
		std::cerr << "#### zDirectionPositionWeight ..... " << options.zDirectionPositionWeight_ << std::endl;
	}
	catch (const std::exception& e){
		options.zDirectionPositionWeight_ = 5.0;
		std::cerr << "#### zDirectionPositionWeight ..... " << options.zDirectionPositionWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.zDirectionVelocityWeight_ = pt.get<double>("model_settings.zDirectionVelocityWeight");
		std::cerr << "#### zDirectionVelocityWeight ..... " << options.zDirectionVelocityWeight_ << std::endl;
	}
	catch (const std::exception& e){
		options.zDirectionVelocityWeight_ = 0.5;
		std::cerr << "#### zDirectionVelocityWeight ..... " << options.zDirectionVelocityWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.swingLegLiftOff_ = pt.get<double>("model_settings.swingLegLiftOff");
		std::cerr << "#### swingLegLiftOff .............. " << options.swingLegLiftOff_ << std::endl;
	}
	catch (const std::exception& e){
		options.swingLegLiftOff_ = 0.3;
		std::cerr << "#### swingLegLiftOff .............. " << options.swingLegLiftOff_ << "\t(default)" << std::endl;
	}

	try {
		options.mpcStrideLength_ = pt.get<double>("model_settings.mpcStrideLength");
		std::cerr << "#### mpcStrideLength .............. " << options.mpcStrideLength_ << std::endl;
	}
	catch (const std::exception& e){
		options.swingLegLiftOff_ = 0.3;
		std::cerr << "#### mpcStrideLength .............. " << options.mpcStrideLength_ << "\t(default)" << std::endl;
	}

	try {
		options.numPhasesInfullGaitCycle_ = pt.get<size_t>("model_settings.numPhasesInfullGaitCycle");
		std::cerr << "#### numPhasesInfullGaitCycle ..... " << options.numPhasesInfullGaitCycle_ << std::endl;
	}
	catch (const std::exception& e){
		options.defaultStartMode_ = Mode::String2ModeNumber("STANCE");
		std::cerr << "#### numPhasesInfullGaitCycle ..... " << options.numPhasesInfullGaitCycle_ << "\t(default)" << std::endl;
	}

	try {
		options.defaultStartMode_ = Mode::String2ModeNumber(pt.get<std::string>("model_settings.defaultStartMode"));
		std::cerr << "#### defaultStartMode ............. " << Mode::ModeNumber2String(options.defaultStartMode_) << std::endl;
	}
	catch (const std::exception& e){
		options.defaultStartMode_ = Mode::String2ModeNumber("STANCE");
		std::cerr << "#### defaultStartMode ............. " << Mode::ModeNumber2String(options.defaultStartMode_) << "\t(default)" << std::endl;
	}

	try {
		options.defaultFinalMode_ = Mode::String2ModeNumber(pt.get<std::string>("model_settings.defaultFinalMode"));
		std::cerr << "#### defaultFinalMode ............. " << Mode::ModeNumber2String(options.defaultFinalMode_) << std::endl;
	}
	catch (const std::exception& e){
		options.defaultFinalMode_ = Mode::String2ModeNumber("STANCE");
		std::cerr << "#### defaultFinalMode ............. " << Mode::ModeNumber2String(options.defaultFinalMode_) << "\t(default)" << std::endl;
	}

	try {
		zmpWeight = pt.get<double>("model_settings.zmpWeight");
		std::cerr << "#### zmpWeight .................... " << zmpWeight << std::endl;
	}
	catch (const std::exception& e){
		zmpWeight = 0.0;
		std::cerr << "#### zmpWeight .................... " << zmpWeight << "\t(default)" << std::endl;
	}

	try {
		impulseWeight = pt.get<double>("model_settings.impulseWeight");
		std::cerr << "#### impulseWeight ................ " << impulseWeight << std::endl;
	}
	catch (const std::exception& e){
		impulseWeight = 0.0;
		std::cerr << "#### impulseWeight ................ " << impulseWeight << "\t(default)" << std::endl;
	}

	try {
		impulseSigmeFactor = pt.get<double>("model_settings.impulseSigmeFactor");
		std::cerr << "#### impulseSigmeFactor ........... " << impulseSigmeFactor << std::endl;
	}
	catch (const std::exception& e){
		impulseSigmeFactor = 1.0/9.0;
		std::cerr << "#### impulseSigmeFactor ........... " << impulseSigmeFactor << "\t(default)" << std::endl;
	}


	std::cerr << std::endl;
}


void loadSwitchingModes(const std::string& filename, std::vector<size_t>& switchingModes, bool verbose = true)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	// read the modes from taskFile
	size_t numSubsystems = 0;
	std::vector<std::string> switchingModesString;
	while (true) {
		try {
			switchingModesString.push_back( pt.get<std::string>("switchingModes.[" + std::to_string(numSubsystems) + "]") );
			numSubsystems++;
		}
		catch (const std::exception& e) {
			break;
		}
	}  // end of while loop

	// convert the mode name to mode enum
	switchingModes.resize(numSubsystems);
	for (size_t i=0; i<numSubsystems; i++)
		switchingModes[i] = Mode::String2ModeNumber(switchingModesString[i]);

	// display
	if (verbose==true) {
		std::cerr << "Switching Modes: " << std::endl;
		std::cerr <<"=====================================================================" << std::endl;
		for (size_t i=0; i<numSubsystems; i++)
			std::cerr << Mode::ModeNumber2String(switchingModes[i]) << ", ";
		std::cerr << std::endl << std::endl;
	}
}


}  // end of anymal namespace

#endif /* LOADTASK_H_ */
