//
// Created by rgrandia on 08.12.21.
//

#pragma once

#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Approximate joint torques with J(q)^T F, i.e. neglecting leg dynamics.
 */
template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> torqueApproximation(const joint_coordinate_s_t<SCALAR_T>& jointPositions,
                                                   const feet_array_t<vector3_s_t<SCALAR_T>>& contactForcesInBase,
                                                   const KinematicsModelBase<SCALAR_T>& kinematics);

// Explicit instantiations
extern template joint_coordinate_s_t<scalar_t> torqueApproximation<scalar_t>(const joint_coordinate_s_t<scalar_t>& jointPositions,
                                                                             const feet_array_t<vector3_s_t<scalar_t>>& contactForcesInBase,
                                                                             const KinematicsModelBase<scalar_t>& kinematics);
extern template joint_coordinate_s_t<ad_scalar_t> torqueApproximation<ad_scalar_t>(
    const joint_coordinate_s_t<ad_scalar_t>& jointPositions, const feet_array_t<vector3_s_t<ad_scalar_t>>& contactForcesInBase,
    const KinematicsModelBase<ad_scalar_t>& kinematics);

}  // namespace switched_model