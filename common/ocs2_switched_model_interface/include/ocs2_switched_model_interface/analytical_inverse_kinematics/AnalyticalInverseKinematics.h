#pragma once

#include "ocs2_switched_model_interface/analytical_inverse_kinematics/LegInverseKinematicParameters.h"

namespace switched_model {
namespace analytical_inverse_kinematics {

/**
 * Computes the inverse kinematics for a single leg given the position in base frame
 * @param legJoints : will be filled with the resulting [HAA, HFE, KFE] joint angles
 * @param positionBaseToFootInBaseFrame : position of the foot in base frame to compute the IK for.
 * @param parameters : precomputed inverse kinematics parameters
 * @param limb : limb number in {LF = 0, RF, LH, RH}
 */
void getLimbJointPositionsFromPositionBaseToFootInBaseFrame(Eigen::Vector3d& legJoints,
                                                            const Eigen::Vector3d& positionBaseToFootInBaseFrame,
                                                            const LegInverseKinematicParameters& parameters, size_t limb);

}  // namespace analytical_inverse_kinematics
}  // namespace switched_model
