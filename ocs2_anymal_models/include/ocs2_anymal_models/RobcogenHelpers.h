//
// Created by rgrandia on 23.09.20.
//

#pragma once

#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/WholebodyDynamics.h>

namespace anymal {
namespace robcogen_helpers {

template <typename INVDYN, typename JSIM, typename SCALAR_T>
typename switched_model::WholebodyDynamics<SCALAR_T>::DynamicsTerms getDynamicsTermsImpl(
    INVDYN& inverseDynamics, JSIM& jsim, const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) {
  const switched_model::base_coordinate_s_t<SCALAR_T> qBase = switched_model::getBasePose(rbdState);
  const switched_model::joint_coordinate_s_t<SCALAR_T> qJoints = switched_model::getJointPositions(rbdState);
  const switched_model::base_coordinate_s_t<SCALAR_T> qdBase = switched_model::getBaseLocalVelocity(rbdState);
  const switched_model::joint_coordinate_s_t<SCALAR_T> qdJoints = switched_model::getJointVelocities(rbdState);

  typename switched_model::WholebodyDynamics<SCALAR_T>::DynamicsTerms dynamicsTerms;
  dynamicsTerms.M = jsim.update(qJoints);

  inverseDynamics.setJointStatus(qJoints);

  {  // Get gravitational vector
    // gravity vector in the base frame
    switched_model::vector6_s_t<SCALAR_T> gravity;
    const auto b_R_o = switched_model::rotationMatrixOriginToBase<SCALAR_T>(switched_model::getOrientation(qBase));
    //! @todo(jcarius) Gravity hardcoded
    const SCALAR_T gravitationalAcceleration = SCALAR_T(9.81);
    gravity << switched_model::vector3_s_t<SCALAR_T>::Zero(),
        b_R_o * switched_model::vector3_s_t<SCALAR_T>(SCALAR_T(0.0), SCALAR_T(0.0), -gravitationalAcceleration);

    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    switched_model::joint_coordinate_s_t<SCALAR_T> jForces;
    inverseDynamics.G_terms_fully_actuated(baseWrench, jForces, gravity);
    dynamicsTerms.G << baseWrench, jForces;
  }

  {  // Get coriolis/centrifugal vector
    switched_model::vector6_s_t<SCALAR_T> baseWrench;
    switched_model::joint_coordinate_s_t<SCALAR_T> jForces;
    inverseDynamics.C_terms_fully_actuated(baseWrench, jForces, qdBase, qdJoints);
    dynamicsTerms.C << baseWrench, jForces;
  }

  return dynamicsTerms;
}

}  // namespace robcogen_helpers
}  // namespace anymal
