/*
 * AnymalWheelsChimeraKinematics.h
 *
 *  Created on: Nov 25, 2019
 *      Author: Marko Bjelonic
 */

#pragma once

#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalWheelsChimeraKinematics final : public switched_model::KinematicsModelBase<SCALAR_T> {
  const SCALAR_T wheelRadius_ = SCALAR_T(0.095);  // [m]

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef switched_model::KinematicsModelBase<SCALAR_T> BASE;
  using typename BASE::joint_jacobian_block_t;

  enum { LF = 0, RF = 1, LH = 2, RH = 3 };

  AnymalWheelsChimeraKinematics() = default;

  ~AnymalWheelsChimeraKinematics() = default;

  AnymalWheelsChimeraKinematics<SCALAR_T>* clone() const override;

  switched_model::vector3_s_t<SCALAR_T> positionBaseToWheelAxisInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  switched_model::matrix3_s_t<SCALAR_T> wheelAxisOrientationInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const;

  switched_model::vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  joint_jacobian_block_t baseToFootJacobianBlockInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  switched_model::matrix3_s_t<SCALAR_T> footOrientationInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;
};

}  // namespace tpl

using AnymalWheelsChimeraKinematics = tpl::AnymalWheelsChimeraKinematics<ocs2::scalar_t>;
using AnymalWheelsChimeraKinematicsAd = tpl::AnymalWheelsChimeraKinematics<ocs2::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::AnymalWheelsChimeraKinematics<ocs2::scalar_t>;
extern template class anymal::tpl::AnymalWheelsChimeraKinematics<ocs2::ad_scalar_t>;
