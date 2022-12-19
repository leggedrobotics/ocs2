//
// Created by rgrandia on 17.02.22.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Configuration for the kinematic penalty taking into account:
 *  - maximum leg extension
 *  - the hip not rotation inwards w.r.t gravity. Prevents placing the foot too far below the body.
 */
struct ApproximateKinematicsConfig {
  scalar_t maxLegExtension;
  scalar_t kinematicPenaltyWeight;
};

/**
 * Compute the kinematic foot placement penalty.
 *
 * @param footPositionInHip : relative foot position in the hip frame
 * @param gravityNormalInHip : gravity vector expressed in the hip frame
 * @param config : settings
 * @return positive kinematic penalty value.
 */
scalar_t computeKinematicPenalty(const vector3_t& footPositionInHip, const vector3_t& gravityNormalInHip,
                                 const ApproximateKinematicsConfig& config);

/**
 * Compute the kinematic foot placement penalty.
 *
 * @param footPositionInWorld : foot position in the world frame
 * @param hipPositionInWorld : hip position in the world frame
 * @param rotationHipToWorld : rotation hip to world. Rotation around the positive x-axis of the hip frame should make the leg rotate
 * outwards.
 * @param config : settings
 * @return positive kinematic penalty value.
 */
scalar_t computeKinematicPenalty(const vector3_t& footPositionInWorld, const vector3_t& hipPositionInWorld,
                                 const matrix3_t& rotationHipToWorld, const ApproximateKinematicsConfig& config);

}  // namespace switched_model