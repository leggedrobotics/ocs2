#ifndef MODEL_SETTINGS_H_
#define MODEL_SETTINGS_H_

#include <vector>
#include <string>
#include <iostream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

class Model_Settings
{
public:
	Model_Settings()
	: constrainedIntegration_(true),
	  gravitationalAcceleration_(9.81),
	  contactForceWeight_(0.1),
	  zDirectionPositionWeight_(5.0),
	  zDirectionVelocityWeight_(0.5),
	  copWeight_(0.0),
	  swingLegLiftOff_(0.3),
	  mpcGoalCommandDelay_(0.5),
	  mpcStrideLength_(0.35),
	  numPhasesInfullGaitCycle_(4)
	 {}

	virtual ~Model_Settings() = default;

	bool constrainedIntegration_;
	double gravitationalAcceleration_;
	double contactForceWeight_;
	double zDirectionPositionWeight_;
	double zDirectionVelocityWeight_;
	double copWeight_;
	double swingLegLiftOff_;
	double mpcGoalCommandDelay_;
	double mpcStrideLength_;
	size_t numPhasesInfullGaitCycle_;

	double eps_ = 0.01;
	double eta_ = 10.0;


	virtual void loadSettings(const std::string& filename, bool verbose = true);
};

inline void Model_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/) {

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	if (verbose)  std::cerr << "\n #### Robot Model Settings:" << std::endl;
	if (verbose)  std::cerr << " #### ==================================================" << std::endl;

	try {
		constrainedIntegration_ = pt.get<bool>("model_settings.constrainedIntegration");
		if (verbose)  std::cerr << " #### constrainedIntegration ....... " << constrainedIntegration_  << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### constrainedIntegration ....... " << constrainedIntegration_  << "\t(default)" << std::endl;
	}

	try {
		gravitationalAcceleration_ = pt.get<double>("model_settings.gravitationalAcceleration");
		if (gravitationalAcceleration_<0) throw std::runtime_error("Gravitational acceleration should be a positive value.");
		if (verbose)  std::cerr << " #### gravitationalAcceleration .... " << gravitationalAcceleration_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### gravitationalAcceleration .... " << gravitationalAcceleration_ << "\t(default)" << std::endl;
	}

	try {
		contactForceWeight_ = pt.get<double>("model_settings.contactForceWeight");
		if (verbose)  std::cerr << " #### contactForceWeight ........... " << contactForceWeight_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### contactForceWeight ........... " << contactForceWeight_ << "\t(default)" << std::endl;
	}

	try {
		zDirectionPositionWeight_ = pt.get<double>("model_settings.zDirectionPositionWeight");
		if (verbose)  std::cerr << " #### zDirectionPositionWeight ..... " << zDirectionPositionWeight_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### zDirectionPositionWeight ..... " << zDirectionPositionWeight_ << "\t(default)" << std::endl;
	}

	try {
		zDirectionVelocityWeight_ = pt.get<double>("model_settings.zDirectionVelocityWeight");
		if (verbose)  std::cerr << " #### zDirectionVelocityWeight ..... " << zDirectionVelocityWeight_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### zDirectionVelocityWeight ..... " << zDirectionVelocityWeight_ << "\t(default)" << std::endl;
	}

	try {
		copWeight_ = pt.get<double>("model_settings.copWeight");
		if (verbose)  std::cerr << " #### copWeight .................... " << copWeight_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### copWeight .................... " << copWeight_ << "\t(default)" << std::endl;
	}

	try {
		swingLegLiftOff_ = pt.get<double>("model_settings.swingLegLiftOff");
		if (verbose)  std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << "\t(default)" << std::endl;
	}

	try {
		mpcGoalCommandDelay_ = pt.get<double>("model_settings.mpcGoalCommandDelay");
		if (verbose)  std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << "\t(default)" << std::endl;
	}

	try {
		mpcStrideLength_ = pt.get<double>("model_settings.mpcStrideLength");
		if (verbose)  std::cerr << " #### mpcStrideLength .............. " << mpcStrideLength_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### mpcStrideLength .............. " << mpcStrideLength_ << "\t(default)" << std::endl;
	}

	try {
		numPhasesInfullGaitCycle_ = pt.get<size_t>("model_settings.numPhasesInfullGaitCycle");
		if (verbose)  std::cerr << " #### numPhasesInfullGaitCycle ..... " << numPhasesInfullGaitCycle_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### numPhasesInfullGaitCycle ..... " << numPhasesInfullGaitCycle_ << "\t(default)" << std::endl;
	}

	try {
		eps_ = pt.get<double>("model_settings.eps");
		if (verbose)  std::cerr << " #### eps .................. " << eps_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### eps .................. " << eps_ << "\t(default)" << std::endl;
	}

	try {
		eta_ = pt.get<double>("model_settings.eta");
		if (verbose)  std::cerr << " #### eta .................. " << eta_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### eta .................. " << eta_ << "\t(default)" << std::endl;
	}

	if(verbose)  std::cerr << " #### ================================================ ####" << std::endl;
}

} // end of namespace switched_model

#endif /* MODEL_SETTINGS_H_ */
