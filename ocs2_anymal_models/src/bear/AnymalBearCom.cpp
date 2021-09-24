/*
 * AnymalBearCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_models/bear/AnymalBearCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include "ocs2_anymal_models/bear/generated/inertia_properties.h"
#include "ocs2_anymal_models/bear/generated/inverse_dynamics.h"
#include "ocs2_anymal_models/bear/generated/jsim.h"
#include "ocs2_anymal_models/bear/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
AnymalBearCom<SCALAR_T>::AnymalBearCom() {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::bear::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  totalMass_ = inertiaProperties_.getTotalMass();
}

template <typename SCALAR_T>
AnymalBearCom<SCALAR_T>* AnymalBearCom<SCALAR_T>::clone() const {
  return new AnymalBearCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> AnymalBearCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::bear::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::bear::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::bear::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::bear::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::bear::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                           jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalBearCom<ocs2::scalar_t>;
template class anymal::tpl::AnymalBearCom<ocs2::ad_scalar_t>;
