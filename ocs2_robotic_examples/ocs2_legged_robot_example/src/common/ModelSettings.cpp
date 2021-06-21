//
// Created by jpsleiman on 26.04.20.
//

#include <ocs2_legged_robot_example/common/ModelSettings.h>

#include <iostream>
#include <string>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2 {
namespace legged_robot {

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
  ocs2::loadData::loadPtreeValue(pt, modelSettings.positionErrorGain_, prefix + "positionErrorGain", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.mpcGoalCommandDelay_, prefix + "mpcGoalCommandDelay", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetDisplacementVelocity_, prefix + "targetDisplacementVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetRotationVelocity_, prefix + "targetRotationVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.gaitOptimization_, prefix + "gaitOptimization", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.enforceFrictionConeConstraint_, prefix + "enforceFrictionConeConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.frictionConeConstraintWeight_, prefix + "frictionConeConstraintWeight", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibraries_, prefix + "recompileLibraries", verbose);

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return modelSettings;
}
}  // namespace legged_robot
}  // namespace ocs2
