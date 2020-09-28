//
// Created by rgrandia on 23.09.20.
//

#include "ocs2_anymal_models/wheels/WholebodyDynamicsWheels.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/wheels/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/wheels/generated/inertia_properties.h"
#include "ocs2_anymal_models/wheels/generated/jsim.h"
#include "ocs2_anymal_models/wheels/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
auto WholebodyDynamicsWheels<SCALAR_T>::getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const -> DynamicsTerms {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::wheels::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::wheels::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::wheels::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::wheels::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::wheels::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  // TODO (rgrandia) : doesn't work for wheels because of the extra wheel joint.
  return anymal::robcogen_helpers::getDynamicsTermsImpl(inverseDynamics_, jointSpaceInertiaMatrix_, rbdState);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::WholebodyDynamicsWheels<ocs2::scalar_t>;
template class anymal::tpl::WholebodyDynamicsWheels<ocs2::ad_scalar_t>;