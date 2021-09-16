/*
 * AnymalChipCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_models/chip/AnymalChipCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/chip/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/chip/generated/inertia_properties.h"
#include "ocs2_anymal_models/chip/generated/jsim.h"
#include "ocs2_anymal_models/chip/generated/miscellaneous.h"
#include "ocs2_anymal_models/chip/generated/transforms.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalChipCom<SCALAR_T>::AnymalChipCom() {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::chip::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  totalMass_ = inertiaProperties_.getTotalMass();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalChipCom<SCALAR_T>* AnymalChipCom<SCALAR_T>::clone() const {
  return new AnymalChipCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> AnymalChipCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::chip::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::chip::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::chip::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::chip::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::chip::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                           jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalChipCom<ocs2::scalar_t>;
template class anymal::tpl::AnymalChipCom<ocs2::ad_scalar_t>;
