//
// Created by rgrandia on 24.10.19.
//

#include "ocs2_switched_model_interface/core/ModelSettings.h"

#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

ModelSettings loadModelSettings(const std::string& filename, bool verbose) {
  ModelSettings modelSettings;

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings."};

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, modelSettings.constrainedIntegration_, prefix + "constrainedIntegration", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.gravitationalAcceleration_, prefix + "gravitationalAcceleration", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.contactForceWeight_, prefix + "contactForceWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.zDirectionPositionWeight_, prefix + "zDirectionPositionWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.zDirectionVelocityWeight_, prefix + "zDirectionVelocityWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.swingLegLiftOff_, prefix + "swingLegLiftOff", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.liftOffVelocity_, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.touchDownVelocity_, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.swingTimeScale_, prefix + "swingTimeScale", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.mpcGoalCommandDelay_, prefix + "mpcGoalCommandDelay", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetDisplacementVelocity_, prefix + "targetDisplacementVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetRotationVelocity_, prefix + "targetRotationVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.useFeetTrajectoryFiltering_, prefix + "useFeetTrajectoryFiltering", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.feetFilterFrequency_, prefix + "feetFilterFrequency", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.gaitOptimization_, prefix + "gaitOptimization", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.enforceFrictionConeConstraint_, prefix + "enforceFrictionConeConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.enforceTorqueConstraint_, prefix + "enforceTorqueConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.torqueLimit_, prefix + "torqueLimit", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibraries_, prefix + "recompileLibraries", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.eps_, prefix + "eps", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.eta_, prefix + "eta", verbose);

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return modelSettings;
}

}  // namespace switched_model