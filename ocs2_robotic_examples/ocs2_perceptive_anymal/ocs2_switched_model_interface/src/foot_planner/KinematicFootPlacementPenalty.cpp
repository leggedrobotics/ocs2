//
// Created by rgrandia on 17.02.22.
//

#include "ocs2_switched_model_interface/foot_planner/KinematicFootPlacementPenalty.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace switched_model {

namespace {
scalar_t computeInwardStepDistance(const vector3_t& footPositionInHip, const vector3_t& gravityNormalInHip) {
  const vector3_t inwardDirection = gravityNormalInHip.cross(vector3_t::UnitX()).normalized();
  return std::max(0.0, inwardDirection.dot(footPositionInHip));
}

scalar_t computeLegOverExtension(const vector3_t& footPositionInHip, const ApproximateKinematicsConfig& config) {
  return std::max(0.0, footPositionInHip.norm() - config.maxLegExtension);
}
}  // namespace

scalar_t computeKinematicPenalty(const vector3_t& footPositionInHip, const vector3_t& gravityNormalInHip,
                                 const ApproximateKinematicsConfig& config) {
  const auto instep = computeInwardStepDistance(footPositionInHip, gravityNormalInHip);
  const auto extension = computeLegOverExtension(footPositionInHip, config);
  return config.kinematicPenaltyWeight * (instep * instep + extension * extension);
}

scalar_t computeKinematicPenalty(const vector3_t& footPositionInWorld, const vector3_t& hipPositionInWorld,
                                 const matrix3_t& rotationHipToWorld, const ApproximateKinematicsConfig& config) {
  const vector3_t footPositionInHip = rotationHipToWorld.transpose() * (footPositionInWorld - hipPositionInWorld);
  const vector3_t gravityNormalInHip = rotationHipToWorld.transpose() * vector3_t(0.0, 0.0, -1.0);
  return computeKinematicPenalty(footPositionInHip, gravityNormalInHip, config);
}

}  // namespace switched_model