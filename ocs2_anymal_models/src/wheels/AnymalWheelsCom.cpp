/*
 * AnymalWheelsCom.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: Marko Bjelonic
 */

#include "ocs2_anymal_models/wheels/AnymalWheelsCom.h"
#include <ocs2_anymal_models/wheels/WheelsSwitchedModel.h>

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include "ocs2_anymal_models/wheels/generated/inverse_dynamics.h"
#include "ocs2_anymal_models/wheels/generated/inertia_properties.h"
#include "ocs2_anymal_models/wheels/generated/jsim.h"
#include "ocs2_anymal_models/wheels/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalWheelsCom<SCALAR_T>::AnymalWheelsCom() {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::wheels::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  totalMass_ = inertiaProperties_.getTotalMass();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalWheelsCom<SCALAR_T>* AnymalWheelsCom<SCALAR_T>::clone() const {
  return new AnymalWheelsCom<SCALAR_T>(*this);
}

template <typename SCALAR_T>
switched_model::base_coordinate_s_t<SCALAR_T> AnymalWheelsCom<SCALAR_T>::calculateBaseLocalAccelerations(
    const switched_model::base_coordinate_s_t<SCALAR_T>& basePose, const switched_model::base_coordinate_s_t<SCALAR_T>& baseLocalVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointVelocities,
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointAccelerations,
    const switched_model::base_coordinate_s_t<SCALAR_T>& forcesOnBaseInBaseFrame) const {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::wheels::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::wheels::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::wheels::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::wheels::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::wheels::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  const auto qExtended = wheels::getExtendedJointCoordinates(jointPositions);

  // use same functions as joint positions, will set wheel velocity and acceleration to 0.0
  const auto dqExtended = wheels::getExtendedJointCoordinates(jointVelocities);
  const auto ddqExtended = wheels::getExtendedJointCoordinates(jointAccelerations);

  return anymal::robcogen_helpers::computeBaseAcceleration(inverseDynamics_, jointSpaceInertiaMatrix_, basePose, baseLocalVelocities,
                                                           qExtended, dqExtended, ddqExtended, forcesOnBaseInBaseFrame);
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalWheelsCom<ocs2::scalar_t>;
template class anymal::tpl::AnymalWheelsCom<ocs2::ad_scalar_t>;
