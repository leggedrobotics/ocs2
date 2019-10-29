//
// Created by rgrandia on 24.10.19.
//

#include "ocs2_switched_model_interface/core/Model_Settings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace switched_model {

void Model_Settings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) std::cerr << "\n #### Robot Model Settings:" << std::endl;
  if (verbose) std::cerr << " #### ==================================================" << std::endl;

  try {
    constrainedIntegration_ = pt.get<bool>("model_settings.constrainedIntegration");
    if (verbose) std::cerr << " #### constrainedIntegration ....... " << constrainedIntegration_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### constrainedIntegration ....... " << constrainedIntegration_ << "\t(default)" << std::endl;
  }

  try {
    gravitationalAcceleration_ = pt.get<double>("model_settings.gravitationalAcceleration");
    if (gravitationalAcceleration_ < 0) throw std::runtime_error("Gravitational acceleration should be a positive value.");
    if (verbose) std::cerr << " #### gravitationalAcceleration .... " << gravitationalAcceleration_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### gravitationalAcceleration .... " << gravitationalAcceleration_ << "\t(default)" << std::endl;
  }

  try {
    contactForceWeight_ = pt.get<double>("model_settings.contactForceWeight");
    if (verbose) std::cerr << " #### contactForceWeight ........... " << contactForceWeight_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### contactForceWeight ........... " << contactForceWeight_ << "\t(default)" << std::endl;
  }

  try {
    zDirectionPositionWeight_ = pt.get<double>("model_settings.zDirectionPositionWeight");
    if (verbose) std::cerr << " #### zDirectionPositionWeight ..... " << zDirectionPositionWeight_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### zDirectionPositionWeight ..... " << zDirectionPositionWeight_ << "\t(default)" << std::endl;
  }

  try {
    zDirectionVelocityWeight_ = pt.get<double>("model_settings.zDirectionVelocityWeight");
    if (verbose) std::cerr << " #### zDirectionVelocityWeight ..... " << zDirectionVelocityWeight_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### zDirectionVelocityWeight ..... " << zDirectionVelocityWeight_ << "\t(default)" << std::endl;
  }

  try {
    swingLegLiftOff_ = pt.get<double>("model_settings.swingLegLiftOff");
    if (verbose) std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### swingLegLiftOff .............. " << swingLegLiftOff_ << "\t(default)" << std::endl;
  }

  try {
    liftOffVelocity_ = pt.get<double>("model_settings.liftOffVelocity");
    if (verbose) std::cerr << " #### liftOffVelocity .............. " << liftOffVelocity_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### liftOffVelocity .............. " << liftOffVelocity_ << "\t(default)" << std::endl;
  }

  try {
    touchDownVelocity_ = pt.get<double>("model_settings.touchDownVelocity");
    if (verbose) std::cerr << " #### touchDownVelocity ............ " << touchDownVelocity_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### touchDownVelocity ............ " << touchDownVelocity_ << "\t(default)" << std::endl;
  }

  try {
    phaseTransitionStanceTime_ = pt.get<double>("model_settings.phaseTransitionStanceTime");
    if (verbose) std::cerr << " #### phaseTransitionStanceTime .... " << phaseTransitionStanceTime_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### phaseTransitionStanceTime .... " << phaseTransitionStanceTime_ << "\t(default)" << std::endl;
  }

  try {
    mpcGoalCommandDelay_ = pt.get<double>("model_settings.mpcGoalCommandDelay");
    if (verbose) std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### mpcGoalCommandDelay .......... " << mpcGoalCommandDelay_ << "\t(default)" << std::endl;
  }

  try {
    targetDisplacementVelocity_ = pt.get<double>("model_settings.targetDisplacementVelocity");
    if (verbose) std::cerr << " #### targetDisplacementVelocity ... " << targetDisplacementVelocity_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### targetDisplacementVelocity ... " << targetDisplacementVelocity_ << "\t(default)" << std::endl;
  }

  try {
    targetRotationVelocity_ = pt.get<double>("model_settings.targetRotationVelocity");
    if (verbose) std::cerr << " #### targetRotationVelocity ....... " << targetRotationVelocity_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### targetRotationVelocity ....... " << targetRotationVelocity_ << "\t(default)" << std::endl;
  }

  try {
    useFeetTrajectoryFiltering_ = pt.get<bool>("model_settings.useFeetTrajectoryFiltering");
    if (verbose) std::cerr << " #### useFeetTrajectoryFiltering ... " << useFeetTrajectoryFiltering_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### useFeetTrajectoryFiltering ... " << useFeetTrajectoryFiltering_ << "\t(default)" << std::endl;
  }

  try {
    feetFilterFrequency_ = pt.get<double>("model_settings.feetFilterFrequency");
    if (verbose) std::cerr << " #### feetFilterFrequency .......... " << feetFilterFrequency_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### feetFilterFrequency .......... " << feetFilterFrequency_ << "\t(default)" << std::endl;
  }

  try {
    torqueMixingFactor_ = pt.get<double>("model_settings.torqueMixingFactor");
    if (verbose) std::cerr << " #### torqueMixingFactor ........... " << torqueMixingFactor_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### torqueMixingFactor ........... " << torqueMixingFactor_ << "\t(default)" << std::endl;
  }

  try {
    gaitOptimization_ = pt.get<bool>("model_settings.gaitOptimization");
    if (verbose) std::cerr << " #### gaitOptimization ............. " << gaitOptimization_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### gaitOptimization ............. " << gaitOptimization_ << "\t(default)" << std::endl;
  }

  try {
    enforceFrictionConeConstraint_ = pt.get<bool>("model_settings.enforceFrictionConeConstraint");
    if (verbose) std::cerr << " #### enforceFrictionConeConstraint  " << enforceFrictionConeConstraint_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### enforceFrictionConeConstraint  " << enforceFrictionConeConstraint_ << "\t(default)" << std::endl;
  }

  try {
    frictionCoefficient_ = pt.get<double>("model_settings.frictionCoefficient");
    if (verbose) std::cerr << " #### frictionCoefficient .......... " << frictionCoefficient_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### frictionCoefficient .......... " << frictionCoefficient_ << "\t(default)" << std::endl;
  }

  try {
    enforceTorqueConstraint_ = pt.get<bool>("model_settings.enforceTorqueConstraint");
    if (verbose) std::cerr << " #### enforceTorqueConstraint ...... " << enforceTorqueConstraint_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### enforceTorqueConstraint ...... " << enforceTorqueConstraint_ << "\t(default)" << std::endl;
  }

  try {
    torqueLimit_ = pt.get<double>("model_settings.torqueLimit");
    if (verbose) std::cerr << " #### torqueLimit .................. " << torqueLimit_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### torqueLimit .................. " << torqueLimit_ << "\t(default)" << std::endl;
  }

  try {
    recompileLibraries_ = pt.get<bool>("model_settings.recompileLibraries");
    if (verbose) std::cerr << " #### recompileLibraries ................. " << recompileLibraries_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### recompileLibraries ................. " << recompileLibraries_ << "\t(default)" << std::endl;
  }

  try {
    eps_ = pt.get<double>("model_settings.eps");
    if (verbose) std::cerr << " #### eps .................. " << eps_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### eps .................. " << eps_ << "\t(default)" << std::endl;
  }

  try {
    eta_ = pt.get<double>("model_settings.eta");
    if (verbose) std::cerr << " #### eta .................. " << eta_ << std::endl;
  } catch (const std::exception& e) {
    if (verbose) std::cerr << " #### eta .................. " << eta_ << "\t(default)" << std::endl;
  }

  if (verbose) std::cerr << " #### ================================================ ####" << std::endl;
}

}