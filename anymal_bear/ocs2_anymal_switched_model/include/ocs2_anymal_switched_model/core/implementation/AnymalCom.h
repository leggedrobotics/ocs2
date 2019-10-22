/*
 * AnymalCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/core/AnymalCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include "ocs2_anymal_switched_model/generated/inertia_properties.h"
#include "ocs2_anymal_switched_model/generated/jsim.h"
#include "ocs2_anymal_switched_model/generated/miscellaneous.h"
#include "ocs2_anymal_switched_model/generated/transforms.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalCom<SCALAR_T>::AnymalCom() {
  joint_coordinate_t defaultJointConfig;
  defaultJointConfig << SCALAR_T(-0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(-0.1),
      SCALAR_T(-0.7), SCALAR_T(1.0), SCALAR_T(0.1), SCALAR_T(-0.7), SCALAR_T(1.0);

  setJointConfiguration(defaultJointConfig);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalCom<SCALAR_T>* AnymalCom<SCALAR_T>::clone() const {
  return new AnymalCom<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void AnymalCom<SCALAR_T>::setJointConfiguration(const joint_coordinate_t& q) {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait ;
  iit::ANYmal::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::ANYmal::tpl::HomogeneousTransforms<trait_t> homTransforms_;
  iit::ANYmal::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::ANYmal::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);

  jointSpaceInertiaMatrix_.update(q);
  comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

  comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
  SCALAR_T& mass = comInertia_(5, 5);
  matrix3s_t crossComPositionBaseFrame = switched_model::CrossProductMatrix<SCALAR_T>(comPositionBaseFrame_);
  comInertia_.template topLeftCorner<3, 3>() -= mass * crossComPositionBaseFrame * crossComPositionBaseFrame.transpose();
  comInertia_.template topRightCorner<3, 3>().setZero();
  comInertia_.template bottomLeftCorner<3, 3>().setZero();

  totalMass_ = inertiaProperties_.getTotalMass();
}

}  // namespace tpl
}  // end of namespace anymal
