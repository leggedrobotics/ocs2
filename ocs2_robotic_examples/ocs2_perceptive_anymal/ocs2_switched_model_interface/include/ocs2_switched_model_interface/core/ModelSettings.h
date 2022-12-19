#pragma once

#include <string>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

enum class Algorithm { DDP, SQP };
std::string toAlgorithmName(Algorithm type);
Algorithm fromAlgorithmName(std::string name);

struct ModelSettings {
  Algorithm algorithm_ = Algorithm::SQP;
  bool recompileLibraries_ = true;
  std::string robotName_ = "quadruped";
  std::string autodiffLibraryFolder_ = "/tmp/ocs2";

  scalar_t phaseTransitionStanceTime_ = 0.4;

  // Analytical IK
  bool analyticalInverseKinematics_ = false;

  // Friction Cone
  scalar_t frictionCoefficient_ = 1.0;
  scalar_t coneRegularization_ = 25.0;
  scalar_t gripperForce_ = 0.0;
  scalar_t muFrictionCone_ = 0.1;
  scalar_t deltaFrictionCone_ = 5.0;

  // FootPlacement relaxed barrier parameters
  scalar_t muFootPlacement_ = 0.1;      // magnitude scaling
  scalar_t deltaFootPlacement_ = 0.01;  // [m] distance from constraint boundary where the barrier becomes quadratic.

  // Collision avoidance relaxed barrier parameters
  scalar_t muSdf_ = 2.5;
  scalar_t deltaSdf_ = 0.005;

  // Joint Limits
  joint_coordinate_t lowerJointLimits_ = joint_coordinate_t::Constant(-1e30);
  joint_coordinate_t upperJointLimits_ = joint_coordinate_t::Constant(1e30);
  joint_coordinate_t jointVelocityLimits = joint_coordinate_t::Constant(1e30);
  joint_coordinate_t jointTorqueLimits = joint_coordinate_t::Constant(1e30);
  scalar_t muJointsPosition_ = 0.1;
  scalar_t deltaJointsPosition_ = 0.1;
  scalar_t muJointsVelocity_ = 0.1;
  scalar_t deltaJointsVelocity_ = 0.1;
  scalar_t muJointsTorque_ = 0.1;
  scalar_t deltaJointsTorque_ = 0.1;
};

ModelSettings loadModelSettings(const std::string& filename, bool verbose = true);

}  // end of namespace switched_model
