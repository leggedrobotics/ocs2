#ifndef OPTIONS_H_
#define OPTIONS_H_

#include <vector>
#include <string>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include "MotionPhaseDefinition.h"

namespace switched_model {

struct Options
{
	Options()
	: constrainedIntegration_(true),
	  contactForceWeight_(0.1),
	  zDirectionPositionWeight_(5.0),
	  zDirectionVelocityWeight_(0.5),
	  copWeight_(0.0),
	  swingLegLiftOff_(0.3),
	  mpcStrideLength_(0.35),
	  numPhasesInfullGaitCycle_(4),
	  defaultStartMode_(15),
	  defaultFinalMode_(15)
	 {}

	bool constrainedIntegration_;
	double contactForceWeight_;
	double zDirectionPositionWeight_;
	double zDirectionVelocityWeight_;
	double copWeight_;
	double swingLegLiftOff_;
	double mpcStrideLength_;
	size_t numPhasesInfullGaitCycle_;
	size_t defaultStartMode_;
	size_t defaultFinalMode_;
};

inline void loadModelSettings(const std::string& filename, Options& options, bool verbose = true) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	std::cerr<<"==========================================" << std::endl;

	try {
		options.constrainedIntegration_ = pt.get<bool>("model_settings.constrainedIntegration");
		std::cerr << "#### constrainedIntegration ....... " << options.constrainedIntegration_  << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### constrainedIntegration ....... " << options.constrainedIntegration_  << "\t(default)" << std::endl;
	}

	try {
		options.contactForceWeight_ = pt.get<double>("model_settings.contactForceWeight");
		std::cerr << "#### contactForceWeight ........... " << options.contactForceWeight_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### contactForceWeight ........... " << options.contactForceWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.zDirectionPositionWeight_ = pt.get<double>("model_settings.zDirectionPositionWeight");
		std::cerr << "#### zDirectionPositionWeight ..... " << options.zDirectionPositionWeight_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### zDirectionPositionWeight ..... " << options.zDirectionPositionWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.zDirectionVelocityWeight_ = pt.get<double>("model_settings.zDirectionVelocityWeight");
		std::cerr << "#### zDirectionVelocityWeight ..... " << options.zDirectionVelocityWeight_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### zDirectionVelocityWeight ..... " << options.zDirectionVelocityWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.copWeight_ = pt.get<double>("model_settings.copWeight");
		std::cerr << "#### copWeight .................... " << options.copWeight_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### copWeight .................... " << options.copWeight_ << "\t(default)" << std::endl;
	}

	try {
		options.swingLegLiftOff_ = pt.get<double>("model_settings.swingLegLiftOff");
		std::cerr << "#### swingLegLiftOff .............. " << options.swingLegLiftOff_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### swingLegLiftOff .............. " << options.swingLegLiftOff_ << "\t(default)" << std::endl;
	}

	try {
		options.mpcStrideLength_ = pt.get<double>("model_settings.mpcStrideLength");
		std::cerr << "#### mpcStrideLength .............. " << options.mpcStrideLength_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### mpcStrideLength .............. " << options.mpcStrideLength_ << "\t(default)" << std::endl;
	}

	try {
		options.numPhasesInfullGaitCycle_ = pt.get<size_t>("model_settings.numPhasesInfullGaitCycle");
		std::cerr << "#### numPhasesInfullGaitCycle ..... " << options.numPhasesInfullGaitCycle_ << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### numPhasesInfullGaitCycle ..... " << options.numPhasesInfullGaitCycle_ << "\t(default)" << std::endl;
	}

	try {
		options.defaultStartMode_ = string2ModeNumber(pt.get<std::string>("model_settings.defaultStartMode"));
		std::cerr << "#### defaultStartMode ............. " << modeNumber2String(options.defaultStartMode_) << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### defaultStartMode ............. " << modeNumber2String(options.defaultStartMode_) << "\t(default)" << std::endl;
	}

	try {
		options.defaultFinalMode_ = string2ModeNumber(pt.get<std::string>("model_settings.defaultFinalMode"));
		std::cerr << "#### defaultFinalMode ............. " << modeNumber2String(options.defaultFinalMode_) << std::endl;
	}
	catch (const std::exception& e){
		std::cerr << "#### defaultFinalMode ............. " << modeNumber2String(options.defaultFinalMode_) << "\t(default)" << std::endl;
	}

	std::cerr << std::endl;
}

} // end of namespace switched_model

#endif /* end of include guard: OPTIONS_H_ */
