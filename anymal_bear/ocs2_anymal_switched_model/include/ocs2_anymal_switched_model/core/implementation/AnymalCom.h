/*
 * AnymalCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/core/AnymalCom.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalCom<SCALAR_T>::AnymalCom()
    : inertiaProperties_(), homTransforms_(), forceTransforms_(), jointSpaceInertiaMatrix_(inertiaProperties_, forceTransforms_) {
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
  jointSpaceInertiaMatrix_.update(q);
  comPositionBaseFrame_ = iit::ANYmal::getWholeBodyCOM(inertiaProperties_, q, homTransforms_);

  comInertia_ = jointSpaceInertiaMatrix_.getWholeBodyInertia();
  SCALAR_T& mass = comInertia_(5, 5);
  matrix3d_t crossComPositionBaseFrame = switched_model::CrossProductMatrix<SCALAR_T>(comPositionBaseFrame_);
  comInertia_.template topLeftCorner<3, 3>() -= mass * crossComPositionBaseFrame * crossComPositionBaseFrame.transpose();
  comInertia_.template topRightCorner<3, 3>().setZero();
  comInertia_.template bottomLeftCorner<3, 3>().setZero();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
typename AnymalCom<SCALAR_T>::vector3d_t AnymalCom<SCALAR_T>::comPositionBaseFrame(const joint_coordinate_t& q) {
  return comPositionBaseFrame_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> AnymalCom<SCALAR_T>::comInertia(const joint_coordinate_t& q) {
  // total inertia of robot in the default config in base frame
  return comInertia_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
SCALAR_T AnymalCom<SCALAR_T>::totalMass() const {
  return inertiaProperties_.getTotalMass();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 4> AnymalCom<SCALAR_T>::comHomogeneous(const joint_coordinate_t& q) {
  Eigen::Matrix<SCALAR_T, 4, 4> res = Eigen::Matrix<SCALAR_T, 4, 4>::Identity();
  res.template topRightCorner<3, 1>() = comPositionBaseFrame(q);
  return res;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 4, 4> AnymalCom<SCALAR_T>::comHomogeneous() {
  Eigen::Matrix<SCALAR_T, 4, 4> res = Eigen::Matrix<SCALAR_T, 4, 4>::Identity();
  res.template topRightCorner<3, 1>() = comPositionBaseFrame();
  return res;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 6> AnymalCom<SCALAR_T>::comInertiaDerivative(const joint_coordinate_t& q, const joint_coordinate_t& dq) {
  return Eigen::Matrix<SCALAR_T, 6, 6>::Zero();  // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 12> AnymalCom<SCALAR_T>::comMomentumJacobian(const joint_coordinate_t& q) {
  return Eigen::Matrix<SCALAR_T, 6, 12>::Zero();  // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 6, 12> AnymalCom<SCALAR_T>::comMomentumJacobianDerivative(const joint_coordinate_t& q,
                                                                                  const joint_coordinate_t& dq) {
  return Eigen::Matrix<SCALAR_T, 6, 12>::Zero();  // massless limbs
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> AnymalCom<SCALAR_T>::comVelocityInBaseFrame(const joint_coordinate_t& q, const joint_coordinate_t& dq) {
  return Eigen::Matrix<SCALAR_T, 3, 1>::Zero();  // massless limbs
}

}  // namespace tpl
}  // end of namespace anymal
