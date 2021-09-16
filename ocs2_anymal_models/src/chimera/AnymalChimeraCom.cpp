/*
 * AnymalChimeraCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_models/chimera/AnymalChimeraCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include "ocs2_anymal_models/chimera/generated/inverse_dynamics.h"
#include "ocs2_anymal_models/chimera/generated/inertia_properties.h"
#include "ocs2_anymal_models/chimera/generated/jsim.h"
#include "ocs2_anymal_models/chimera/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalChimeraCom<SCALAR_T>::AnymalChimeraCom() {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::chimera::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  totalMass_ = inertiaProperties_.getTotalMass();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalChimeraCom<SCALAR_T>* AnymalChimeraCom<SCALAR_T>::clone() const {
  return new AnymalChimeraCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> AnymalChimeraCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::chimera::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::chimera::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::chimera::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::chimera::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::chimera::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                           jointPositions, jointVelocities, jointAccelerations, forcesOnBaseInBaseFrame);
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalChimeraCom<ocs2::scalar_t>;
template class anymal::tpl::AnymalChimeraCom<ocs2::ad_scalar_t>;
