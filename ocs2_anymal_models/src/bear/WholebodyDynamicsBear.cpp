//
// Created by rgrandia on 23.09.20.
//

#include "ocs2_anymal_models/bear/WholebodyDynamicsBear.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/bear/generated/inverse_dynamics.h>
#include <ocs2_anymal_models/robcogenHelper.h>
#include "ocs2_anymal_models/bear/generated/inertia_properties.h"
#include "ocs2_anymal_models/bear/generated/jsim.h"
#include "ocs2_anymal_models/bear/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
auto WholebodyDynamicsBear<SCALAR_T>::getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const -> DynamicsTerms {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::bear::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::bear::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::bear::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::bear::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::bear::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::getDynamicsTermsImpl(inertiaProperties_, jointSpaceInertiaMatrix_, rbdState);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::WholebodyDynamicsBear<ocs2::scalar_t>;
template class anymal::tpl::WholebodyDynamicsBear<ocs2::ad_scalar_t>;