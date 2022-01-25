//
// Created by rgrandia on 23.09.20.
//

#include "ocs2_anymal_models/camel/WholebodyDynamicsCamel.h"

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_anymal_models/RobcogenHelpers.h>
#include <ocs2_anymal_models/camel/generated/inverse_dynamics.h>
#include "ocs2_anymal_models/camel/generated/inertia_properties.h"
#include "ocs2_anymal_models/camel/generated/jsim.h"
#include "ocs2_anymal_models/camel/generated/transforms.h"

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
auto WholebodyDynamicsCamel<SCALAR_T>::getDynamicsTerms(const switched_model::rbd_state_s_t<SCALAR_T>& rbdState) const -> DynamicsTerms {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::camel::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::camel::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::camel::tpl::MotionTransforms<trait_t> motionTransforms_;
  iit::camel::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);
  iit::camel::dyn::tpl::InverseDynamics<trait_t> inverseDynamics_(inertiaProperties_, motionTransforms_);

  return anymal::robcogen_helpers::getDynamicsTermsImpl(inverseDynamics_, jointSpaceInertiaMatrix_, rbdState);
}

}  // namespace tpl
}  // namespace anymal

// Explicit instantiation
template class anymal::tpl::WholebodyDynamicsCamel<ocs2::scalar_t>;
template class anymal::tpl::WholebodyDynamicsCamel<ocs2::ad_scalar_t>;