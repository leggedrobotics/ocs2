/*
 * AnymalBearCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_bear_switched_model/core/AnymalBearCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include "ocs2_anymal_bear_switched_model/generated/inertia_properties.h"
#include "ocs2_anymal_bear_switched_model/generated/jsim.h"
#include "ocs2_anymal_bear_switched_model/generated/miscellaneous.h"
#include "ocs2_anymal_bear_switched_model/generated/transforms.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalBearCom<SCALAR_T>::AnymalBearCom() {
  switched_model::joint_coordinate_s_t<SCALAR_T> defaultJointConfig;
  defaultJointConfig << SCALAR_T(-0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(-0.1),
      SCALAR_T(-0.7), SCALAR_T(1.0), SCALAR_T(0.1), SCALAR_T(-0.7), SCALAR_T(1.0);

  setJointConfiguration(defaultJointConfig);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalBearCom<SCALAR_T>* AnymalBearCom<SCALAR_T>::clone() const {
  return new AnymalBearCom<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void AnymalBearCom<SCALAR_T>::setJointConfiguration(const switched_model::joint_coordinate_s_t<SCALAR_T>& q) {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::ANYmal::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::ANYmal::tpl::HomogeneousTransforms<trait_t> homTransforms_;
  iit::ANYmal::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::ANYmal::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);

  jointSpaceInertiaMatrix_.update(q);
  comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

  comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
  SCALAR_T mass = comInertia_(5, 5);
  switched_model::matrix3_s_t<SCALAR_T> crossComPositionBaseFrame = switched_model::crossProductMatrix<SCALAR_T>(comPositionBaseFrame_);
  comInertia_.template topLeftCorner<3, 3>() -= mass * crossComPositionBaseFrame * crossComPositionBaseFrame.transpose();
  comInertia_.template topRightCorner<3, 3>().setZero();
  comInertia_.template bottomLeftCorner<3, 3>().setZero();

  totalMass_ = inertiaProperties_.getTotalMass();
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalBearCom<ocs2::scalar_t>;
template class anymal::tpl::AnymalBearCom<ocs2::CppAdInterface::ad_scalar_t>;
