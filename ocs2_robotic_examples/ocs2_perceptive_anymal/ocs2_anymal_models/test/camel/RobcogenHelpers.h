//
// Created by rgrandia on 23.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include "ocs2_anymal_models/DynamicsHelpers.h"

namespace anymal {
namespace robcogen_helpers {

template <typename SCALAR_T, int NUM_JOINTS_IMPL>
struct BaseDynamicsTerms {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /// 6 x 6 : inertia tensor of the base
  Eigen::Matrix<SCALAR_T, switched_model::BASE_COORDINATE_SIZE, switched_model::BASE_COORDINATE_SIZE> Mb;
  /// 6 x number joints : inertial coupling between joints and base
  Eigen::Matrix<SCALAR_T, switched_model::BASE_COORDINATE_SIZE, NUM_JOINTS_IMPL> Mj;
  /// centrifugal/coriolis effects on the base
  switched_model::base_coordinate_s_t<SCALAR_T> C;
  /// result of inv(Mb) * G, where G is the gravitation base term of the rigid body dynamics.
  switched_model::base_coordinate_s_t<SCALAR_T> invMbG;
};

template <typename INVDYN, typename JSIM, typename SCALAR_T, int NUM_JOINTS_IMPL>
BaseDynamicsTerms<SCALAR_T, NUM_JOINTS_IMPL> getBaseDynamicsTermsImpl(INVDYN& inverseDynamics, JSIM& jsim,
                                                                      const switched_model::base_coordinate_s_t<SCALAR_T>& qBase,
                                                                      const switched_model::base_coordinate_s_t<SCALAR_T>& qdBase,
                                                                      const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& qJoints,
                                                                      const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& qdJoints) {
  BaseDynamicsTerms<SCALAR_T, NUM_JOINTS_IMPL> baseDynamicsTerms;

  const auto M = jsim.update(qJoints);
  baseDynamicsTerms.Mb = M.template topLeftCorner<switched_model::BASE_COORDINATE_SIZE, switched_model::BASE_COORDINATE_SIZE>();
  baseDynamicsTerms.Mj = M.template topRightCorner<switched_model::BASE_COORDINATE_SIZE, NUM_JOINTS_IMPL>();

  inverseDynamics.setJointStatus(qJoints);

  {  // Get gravitational vector
    // gravity vector in the base frame
    const SCALAR_T gravitationalAcceleration = SCALAR_T(9.81);
    baseDynamicsTerms.invMbG << switched_model::vector3_s_t<SCALAR_T>::Zero(),
        switched_model::rotateVectorOriginToBase(
            switched_model::vector3_s_t<SCALAR_T>(SCALAR_T(0.0), SCALAR_T(0.0), gravitationalAcceleration),
            switched_model::getOrientation(qBase));
  }

  {  // Get coriolis/centrifugal vector
    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1> jForces;
    inverseDynamics.C_terms_fully_actuated(baseDynamicsTerms.C, jForces, qdBase, qdJoints);
  }

  return baseDynamicsTerms;
}

template <typename INVDYN, typename JSIM, typename SCALAR_T, int NUM_JOINTS_IMPL>
switched_model::base_coordinate_s_t<SCALAR_T> computeBaseAcceleration(
    INVDYN& inverseDynamics, JSIM& jsim, const switched_model::base_coordinate_s_t<SCALAR_T>& basePose,
    const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointPositions, const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointVelocities,
    const Eigen::Matrix<SCALAR_T, NUM_JOINTS_IMPL, 1>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) {
  const auto baseDynamics = getBaseDynamicsTermsImpl(inverseDynamics, jsim, basePose, baseLocalVelocities, jointPositions, jointVelocities);

  switched_model::base_coordinate_s_t<SCALAR_T> baseForces = forcesOnBaseInBaseFrame - baseDynamics.C;
  baseForces.noalias() -= baseDynamics.Mj * jointAccelerations;

  return inertiaTensorSolveAngularLinear<SCALAR_T>(baseDynamics.Mb, baseForces) - baseDynamics.invMbG;
}

}  // namespace robcogen_helpers
}  // namespace anymal
