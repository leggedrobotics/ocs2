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
	: constrainedIntegration_(true)
	, gravitationalAcceleration_(9.81)
	, contactForceWeight_(0.1)
	, zDirectionPositionWeight_(5.0)
	, zDirectionVelocityWeight_(0.5)
	, swingLegLiftOff_(0.3)
	, liftOffVelocity_(0.0)
	, touchDownVelocity_(0.0)
	, phaseTransitionStanceTime_(0.4)
	, mpcGoalCommandDelay_(0.5)
	, targetDisplacementVelocity_(0.5)
	, targetRotationVelocity_(0.3)
	, useFeetTrajectoryFiltering_(true)
	, feetFilterFrequency_(50.0)
	, torqueMixingFactor_(0.0)
	, gaitOptimization_(false)
	, enforceFrictionConeConstraint_(false)
	, frictionCoefficient_(1.0)
	, enforceTorqueConstraint_(false)
	, torqueLimit_(40.0)
	{}

	virtual ~Model_Settings() = default;

	bool constrainedIntegration_;
	double gravitationalAcceleration_;
	double contactForceWeight_;
	double zDirectionPositionWeight_;
	double zDirectionVelocityWeight_;
	double swingLegLiftOff_;
	double liftOffVelocity_;
	double touchDownVelocity_;
	double phaseTransitionStanceTime_;
	double mpcGoalCommandDelay_;
	double targetDisplacementVelocity_;
	double targetRotationVelocity_;
	bool useFeetTrajectoryFiltering_;
	double feetFilterFrequency_;
	double torqueMixingFactor_;
	bool gaitOptimization_;
	bool enforceFrictionConeConstraint_;
	double frictionCoefficient_;
	bool enforceTorqueConstraint_;
	double torqueLimit_;

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
		swingLegLiftOff_ = pt.get<double>("model_settings.swingLegLiftOff");
		if (verbose)  std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << "\t(default)" << std::endl;
	}

	try {
		liftOffVelocity_ = pt.get<double>("model_settings.liftOffVelocity");
		if (verbose)  std::cerr << " #### liftOffVelocity .............. " << liftOffVelocity_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### liftOffVelocity .............. " << liftOffVelocity_ << "\t(default)" << std::endl;
	}

	try {
		touchDownVelocity_ = pt.get<double>("model_settings.touchDownVelocity");
		if (verbose)  std::cerr << " #### touchDownVelocity ............ " << touchDownVelocity_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### touchDownVelocity ............ " << touchDownVelocity_ << "\t(default)" << std::endl;
	}

	try {
		phaseTransitionStanceTime_ = pt.get<double>("model_settings.phaseTransitionStanceTime");
		if (verbose)  std::cerr << " #### phaseTransitionStanceTime .... " << phaseTransitionStanceTime_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### phaseTransitionStanceTime .... " << phaseTransitionStanceTime_ << "\t(default)" << std::endl;
	}

	try {
		mpcGoalCommandDelay_ = pt.get<double>("model_settings.mpcGoalCommandDelay");
		if (verbose)  std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << "\t(default)" << std::endl;
	}

	try {
		targetDisplacementVelocity_ = pt.get<double>("model_settings.targetDisplacementVelocity");
		if (verbose)  std::cerr << " #### targetDisplacementVelocity ... " << targetDisplacementVelocity_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### targetDisplacementVelocity ... " << targetDisplacementVelocity_ << "\t(default)" << std::endl;
	}

	try {
		targetRotationVelocity_ = pt.get<double>("model_settings.targetRotationVelocity");
		if (verbose)  std::cerr << " #### targetRotationVelocity ....... " << targetRotationVelocity_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### targetRotationVelocity ....... " << targetRotationVelocity_ << "\t(default)" << std::endl;
	}

	try {
		useFeetTrajectoryFiltering_ = pt.get<bool>("model_settings.useFeetTrajectoryFiltering");
		if (verbose)  std::cerr << " #### useFeetTrajectoryFiltering ... " << useFeetTrajectoryFiltering_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### useFeetTrajectoryFiltering ... " << useFeetTrajectoryFiltering_ << "\t(default)" << std::endl;
	}

	try {
		feetFilterFrequency_ = pt.get<double>("model_settings.feetFilterFrequency");
		if (verbose)  std::cerr << " #### feetFilterFrequency .......... " << feetFilterFrequency_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### feetFilterFrequency .......... " << feetFilterFrequency_ << "\t(default)" << std::endl;
	}

	try {
		torqueMixingFactor_ = pt.get<double>("model_settings.torqueMixingFactor");
		if (verbose)  std::cerr << " #### torqueMixingFactor ........... " << torqueMixingFactor_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### torqueMixingFactor ........... " << torqueMixingFactor_ << "\t(default)" << std::endl;
	}

	try {
		gaitOptimization_ = pt.get<bool>("model_settings.gaitOptimization");
		if (verbose)  std::cerr << " #### gaitOptimization ............. " << gaitOptimization_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### gaitOptimization ............. " << gaitOptimization_ << "\t(default)" << std::endl;
	}

	try {
		enforceFrictionConeConstraint_ = pt.get<bool>("model_settings.enforceFrictionConeConstraint");
		if (verbose)  std::cerr << " #### enforceFrictionConeConstraint  " << enforceFrictionConeConstraint_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### enforceFrictionConeConstraint  " << enforceFrictionConeConstraint_ << "\t(default)" << std::endl;
	}

	try {
		frictionCoefficient_ = pt.get<double>("model_settings.frictionCoefficient");
		if (verbose)  std::cerr << " #### frictionCoefficient .......... " << frictionCoefficient_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### frictionCoefficient .......... " << frictionCoefficient_ << "\t(default)" << std::endl;
	}

	try {
		enforceTorqueConstraint_ = pt.get<bool>("model_settings.enforceTorqueConstraint");
		if (verbose)  std::cerr << " #### enforceTorqueConstraint ...... " << enforceTorqueConstraint_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### enforceTorqueConstraint ...... " << enforceTorqueConstraint_ << "\t(default)" << std::endl;
	}

	try {
		torqueLimit_ = pt.get<double>("model_settings.torqueLimit");
		if (verbose)  std::cerr << " #### torqueLimit .................. " << torqueLimit_ << std::endl;
	}
	catch (const std::exception& e){
		if (verbose)  std::cerr << " #### torqueLimit .................. " << torqueLimit_ << "\t(default)" << std::endl;
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
