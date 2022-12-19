//
// Created by rgrandia on 08.12.21.
//

#include "ocs2_switched_model_interface/core/TorqueApproximation.h"

namespace switched_model {

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> torqueApproximation(const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                   const feet_array_t<vector3_s_t<SCALAR_T>>& contactForcesInBase,
                                                   const KinematicsModelBase<SCALAR_T>& kinematics) {
  joint_coordinate_s_t<SCALAR_T> torques;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    // Part of the foot Jacobian that corresponds to the joints in this leg.
    const auto b_baseToFootJacobianBlock = kinematics.baseToFootJacobianBlockInBaseFrame(leg, jointPositions);

    // tau = -J^T * F, (bottom 3 rows = translational part)
    torques.template segment<3>(3 * leg) = -b_baseToFootJacobianBlock.bottomRows(3).transpose() * contactForcesInBase[leg];
  }
  return torques;
}

template joint_coordinate_s_t<scalar_t> torqueApproximation<scalar_t>(const joint_coordinate_s_t<scalar_t>& jointPositions,
                                                                      const feet_array_t<vector3_s_t<scalar_t>>& contactForcesInBase,
                                                                      const KinematicsModelBase<scalar_t>& kinematics);
template joint_coordinate_s_t<ad_scalar_t> torqueApproximation<ad_scalar_t>(
    const joint_coordinate_s_t<ad_scalar_t>& jointPositions, const feet_array_t<vector3_s_t<ad_scalar_t>>& contactForcesInBase,
    const KinematicsModelBase<ad_scalar_t>& kinematics);

}  // namespace switched_model