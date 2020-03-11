/*
 * AnymalWheelsCom.cpp
 *
 *  Created on: Nov 25, 2019
 *      Author: Marko Bjelonic
 */

#include "ocs2_anymal_wheels_switched_model/core/AnymalWheelsCom.h"

#include <iit/rbd/traits/TraitSelector.h>
#include "ocs2_anymal_wheels_switched_model/generated/inertia_properties.h"
#include "ocs2_anymal_wheels_switched_model/generated/jsim.h"
#include "ocs2_anymal_wheels_switched_model/generated/miscellaneous.h"
#include "ocs2_anymal_wheels_switched_model/generated/transforms.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalWheelsCom<SCALAR_T>::AnymalWheelsCom() {
  switched_model::joint_coordinate_s_t<SCALAR_T> defaultJointConfig;
  defaultJointConfig << SCALAR_T(-0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(0.1), SCALAR_T(0.7), SCALAR_T(-1.0), SCALAR_T(-0.1),
      SCALAR_T(-0.7), SCALAR_T(1.0), SCALAR_T(0.1), SCALAR_T(-0.7), SCALAR_T(1.0);

  setJointConfiguration(defaultJointConfig);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalWheelsCom<SCALAR_T>* AnymalWheelsCom<SCALAR_T>::clone() const {
  return new AnymalWheelsCom<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void AnymalWheelsCom<SCALAR_T>::setJointConfiguration(const switched_model::joint_coordinate_s_t<SCALAR_T>& q) {
  using trait_t = typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait;
  iit::ANYmal::dyn::tpl::InertiaProperties<trait_t> inertiaProperties_;
  iit::ANYmal::tpl::HomogeneousTransforms<trait_t> homTransforms_;
  iit::ANYmal::tpl::ForceTransforms<trait_t> forceTransforms_;
  iit::ANYmal::dyn::tpl::JSIM<trait_t> jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_);

  const auto qExtended = getExtendedJointCoordinates(q);
  jointSpaceInertiaMatrix_.update(qExtended);
  comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_, qExtended, homTransforms_);

  comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
  SCALAR_T mass = comInertia_(5, 5);
  switched_model::matrix3_s_t<SCALAR_T> crossComPositionBaseFrame = switched_model::crossProductMatrix<SCALAR_T>(comPositionBaseFrame_);
  comInertia_.template topLeftCorner<3, 3>() -= mass * crossComPositionBaseFrame * crossComPositionBaseFrame.transpose();
  comInertia_.template topRightCorner<3, 3>().setZero();
  comInertia_.template bottomLeftCorner<3, 3>().setZero();

  totalMass_ = inertiaProperties_.getTotalMass();
}

template <typename SCALAR_T>
typename AnymalWheelsCom<SCALAR_T>::extended_joint_coordinate_t AnymalWheelsCom<SCALAR_T>::getExtendedJointCoordinates(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
  typename AnymalWheelsCom<SCALAR_T>::extended_joint_coordinate_t extendedJointCoordinate;
  extendedJointCoordinate.template segment<3>(0) = jointPositions.template segment<3>(0);
  extendedJointCoordinate(3) = SCALAR_T(0.0);
  extendedJointCoordinate.template segment<3>(4) = jointPositions.template segment<3>(3);
  extendedJointCoordinate(7) = SCALAR_T(0.0);
  extendedJointCoordinate.template segment<3>(8) = jointPositions.template segment<3>(6);
  extendedJointCoordinate(11) = SCALAR_T(0.0);
  extendedJointCoordinate.template segment<3>(12) = jointPositions.template segment<3>(9);
  extendedJointCoordinate(15) = SCALAR_T(0.0);
  return extendedJointCoordinate;
}

}  // namespace tpl
}  // end of namespace anymal

// Explicit instantiation
template class anymal::tpl::AnymalWheelsCom<double>;
template class anymal::tpl::AnymalWheelsCom<ocs2::CppAdInterface<double>::ad_scalar_t>;
