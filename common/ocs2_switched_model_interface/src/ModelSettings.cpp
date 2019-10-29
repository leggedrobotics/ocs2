//
// Created by rgrandia on 24.10.19.
//

#include "ocs2_switched_model_interface/core/ModelSettings.h"

#include <ocs2_core/misc/LoadData.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace switched_model {

void ModelSettings::loadSettings(const std::string& filename, bool verbose /*= true*/) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const std::string prefix{"model_settings."};

  if (verbose) {
    std::cerr << "\n #### Robot Model Settings:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  ocs2::loadData::loadPtreeValue(pt, constrainedIntegration_, prefix + "constrainedIntegration", verbose);
  ocs2::loadData::loadPtreeValue(pt, gravitationalAcceleration_, prefix + "gravitationalAcceleration", verbose);
  ocs2::loadData::loadPtreeValue(pt, contactForceWeight_, prefix + "contactForceWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, zDirectionPositionWeight_, prefix + "zDirectionPositionWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, zDirectionVelocityWeight_, prefix + "zDirectionVelocityWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, swingLegLiftOff_, prefix + "swingLegLiftOff", verbose);
  ocs2::loadData::loadPtreeValue(pt, liftOffVelocity_, prefix + "liftOffVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, touchDownVelocity_, prefix + "touchDownVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, mpcGoalCommandDelay_, prefix + "mpcGoalCommandDelay", verbose);
  ocs2::loadData::loadPtreeValue(pt, targetDisplacementVelocity_, prefix + "targetDisplacementVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, targetRotationVelocity_, prefix + "targetRotationVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, useFeetTrajectoryFiltering_, prefix + "useFeetTrajectoryFiltering", verbose);
  ocs2::loadData::loadPtreeValue(pt, feetFilterFrequency_, prefix + "feetFilterFrequency", verbose);
  ocs2::loadData::loadPtreeValue(pt, torqueMixingFactor_, prefix + "torqueMixingFactor", verbose);
  ocs2::loadData::loadPtreeValue(pt, gaitOptimization_, prefix + "gaitOptimization", verbose);
  ocs2::loadData::loadPtreeValue(pt, enforceFrictionConeConstraint_, prefix + "enforceFrictionConeConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, enforceTorqueConstraint_, prefix + "enforceTorqueConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, torqueLimit_, prefix + "torqueLimit", verbose);
  ocs2::loadData::loadPtreeValue(pt, recompileLibraries_, prefix + "recompileLibraries", verbose);
  ocs2::loadData::loadPtreeValue(pt, eps_, prefix + "eps", verbose);
  ocs2::loadData::loadPtreeValue(pt, eta_, prefix + "eta", verbose);

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }
}

}  // namespace switched_model