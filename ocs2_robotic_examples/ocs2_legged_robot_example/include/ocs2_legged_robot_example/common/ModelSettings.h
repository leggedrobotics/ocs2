//
// Created by jpsleiman on 26.04.20.
//
#pragma once

#include <string>

#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_legged_robot_example/common/definitions.h"

namespace ocs2 {
namespace legged_robot {

struct ModelSettings {
  bool constrainedIntegration_ = true;
  scalar_t gravitationalAcceleration_ = 9.81;
  scalar_t contactForceWeight_ = 0.1;
  scalar_t zDirectionPositionWeight_ = 5.0;
  scalar_t zDirectionVelocityWeight_ = 0.5;
  scalar_t positionErrorGain_ = 0.5;
  scalar_t phaseTransitionStanceTime_ = 0.4;
  scalar_t mpcGoalCommandDelay_ = 0.5;
  scalar_t targetDisplacementVelocity_ = 0.5;
  scalar_t targetRotationVelocity_ = 0.3;
  bool enforceFrictionConeConstraint_ = false;
  scalar_t frictionCoefficient_ = 1.0;
  scalar_t frictionConeConstraintWeight_ = 1.0;
  bool recompileLibraries_ = true;
};

ModelSettings loadModelSettings(const std::string& filename, bool verbose = "true");

}  // namespace legged_robot
}  // namespace ocs2
