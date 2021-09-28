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

  std::string algorithmName = toAlgorithmName(modelSettings.algorithm_);
  ocs2::loadData::loadPtreeValue(pt, algorithmName, prefix + "algorithm", verbose);
  modelSettings.algorithm_ = fromAlgorithmName(algorithmName);

  ocs2::loadData::loadPtreeValue(pt, modelSettings.phaseTransitionStanceTime_, prefix + "phaseTransitionStanceTime", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.simplifyDynamics_, prefix + "simplifyDynamics", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.recompileLibraries_, prefix + "recompileLibraries", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.frictionCoefficient_, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.muFrictionCone_, prefix + "muFrictionCone", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.deltaFrictionCone_, prefix + "deltaFrictionCone", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.muFootPlacement_, prefix + "muFootPlacement", verbose);
  ocs2::loadData::loadPtreeValue(pt, modelSettings.deltaFootPlacement_, prefix + "deltaFootPlacement", verbose);
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

  if (verbose) {
    std::cerr << " #### ================================================ ####" << std::endl;
  }

  return modelSettings;
}

}  // namespace switched_model
