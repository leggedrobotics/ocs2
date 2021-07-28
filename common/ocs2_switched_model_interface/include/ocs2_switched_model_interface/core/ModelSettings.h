#pragma once

#include <string>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

enum class Algorithm { DDP, SQP };
std::string toAlgorithmName(Algorithm type);
Algorithm fromAlgorithmName(std::string name);

struct ModelSettings {
  bool constrainedIntegration_ = true;
  scalar_t gravitationalAcceleration_ = 9.81;
  scalar_t contactForceWeight_ = 0.1;
  scalar_t zDirectionPositionWeight_ = 5.0;
  scalar_t zDirectionVelocityWeight_ = 0.5;
  scalar_t phaseTransitionStanceTime_ = 0.4;
  scalar_t mpcGoalCommandDelay_ = 0.5;
  scalar_t targetDisplacementVelocity_ = 0.5;
  scalar_t targetRotationVelocity_ = 0.3;
  bool gaitOptimization_ = false;
  bool enforceFrictionConeConstraint_ = false;
  scalar_t frictionCoefficient_ = 1.0;
  bool enforceTorqueConstraint_ = false;
  scalar_t torqueLimit_ = 40.0;
  bool recompileLibraries_ = true;

  // FootPlacementCostParameters
  scalar_t mu_ = 0.1;      // magnitude scaling
  scalar_t delta_ = 0.01;  // [m] distance from constraint boundary where the barrier becomes quadratic.

  // FootPlacementCostParameters Sdf
  scalar_t muSdf_ = 2.5;
  scalar_t deltaSdf_ = 0.005;

  // Joint Limits
  joint_coordinate_t lowerJointLimits_ = joint_coordinate_t::Constant(-1e30);
  joint_coordinate_t upperJointLimits_ = joint_coordinate_t::Constant(1e30);
  scalar_t muJoints_ = 0.1;
  scalar_t deltaJoints_ = 0.1;

  Algorithm algorithm_ = Algorithm::SQP;
};

ModelSettings loadModelSettings(const std::string& filename, bool verbose = true);

}  // end of namespace switched_model
