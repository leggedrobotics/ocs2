//
// Created by rgrandia on 24.10.19.
//

#include "ocs2_switched_model_interface/core/ModelSettings.h"

#include <iostream>
#include <unordered_map>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

std::string toAlgorithmName(Algorithm type) {
  static const std::unordered_map<Algorithm, std::string> map{{Algorithm::DDP, "DDP"}, {Algorithm::SQP, "SQP"}};
  return map.at(type);
}

Algorithm fromAlgorithmName(std::string name) {
  static const std::unordered_map<std::string, Algorithm> map{{"DDP", Algorithm::DDP}, {"SQP", Algorithm::SQP}};
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  return map.at(name);
}

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
  ocs2::loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.mpcGoalCommandDelay_, prefix + "mpcGoalCommandDelay", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetDisplacementVelocity_, prefix + "targetDisplacementVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.targetRotationVelocity_, prefix + "targetRotationVelocity", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.gaitOptimization_, prefix + "gaitOptimization", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.enforceFrictionConeConstraint_, prefix + "enforceFrictionConeConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.enforceTorqueConstraint_, prefix + "enforceTorqueConstraint", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.torqueLimit_, prefix + "torqueLimit", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibraries_, prefix + "recompileLibraries", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.mu_, prefix + "mu", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.delta_, prefix + "delta", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.muSdf_, prefix + "muSdf", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.deltaSdf_, prefix + "deltaSdf", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.muJoints_, prefix + "muJoints", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.deltaJoints_, prefix + "deltaJoints", verbose);
  ocs2::loadData::loadEigenMatrix(filename, prefix + "joint_lower_limits", modelSettings.lowerJointLimits_);
  ocs2::loadData::loadEigenMatrix(filename, prefix + "joint_upper_limits", modelSettings.upperJointLimits_);

  if (verbose) {
    std::cerr << " joint lower limits: " << modelSettings.lowerJointLimits_.transpose() << "\n";
    std::cerr << " joint upper limits: " << modelSettings.upperJointLimits_.transpose() << "\n";
  }

  std::string algorithmName = toAlgorithmName(modelSettings.algorithm_);
  ocs2::loadData::loadPtreeValue(pt, algorithmName, prefix + "algorithm", verbose);
  modelSettings.algorithm_ = fromAlgorithmName(algorithmName);

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return modelSettings;
}

}  // namespace switched_model
