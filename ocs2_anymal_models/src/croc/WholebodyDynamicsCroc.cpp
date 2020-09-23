//
// Created by rgrandia on 23.09.20.
//

#include "ocs2_anymal_models/croc/WholebodyDynamicsCroc.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/croc/generated/inverse_dynamics.h>
#include <ocs2_anymal_models/robcogenHelper.h>
#include "ocs2_anymal_models/croc/generated/inertia_properties.h"
#include "ocs2_anymal_models/croc/generated/jsim.h"
#include "ocs2_anymal_models/croc/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
auto WholebodyDynamicsCroc<SCALAR_T>::getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const -> DynamicsTerms {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::croc::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::croc::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::croc::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::croc::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::croc::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::getDynamicsTermsImpl(inertiaProperties_, jointSpaceInertiaMatrix_, rbdState);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::WholebodyDynamicsCroc<ocs2::scalar_t>;
template class anymal::tpl::WholebodyDynamicsCroc<ocs2::ad_scalar_t>;