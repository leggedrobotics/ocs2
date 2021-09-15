//
// Created by rgrandia on 23.09.20.
//

#include "ocs2_anymal_models/chip/WholebodyDynamicsChip.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/chip/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/chip/generated/inertia_properties.h"
#include "ocs2_anymal_models/chip/generated/jsim.h"
#include "ocs2_anymal_models/chip/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
auto WholebodyDynamicsChip<SCALAR_T>::getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const -> DynamicsTerms {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::chip::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::chip::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::chip::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::chip::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::chip::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::getDynamicsTermsImpl(inverseDynamics_, jointSpaceInertiaMatrix_, rbdState);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::WholebodyDynamicsChip<ocs2::scalar_t>;
template class anymal::tpl::WholebodyDynamicsChip<ocs2::ad_scalar_t>;