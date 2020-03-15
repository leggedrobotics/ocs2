/*
 * AnymalCrocKinematics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalCrocKinematics final : public switched_model::KinematicsModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef switched_model::KinematicsModelBase<SCALAR_T> BASE;
  using typename BASE::joint_jacobian_t;

  enum { LF = 0, RF = 1, LH = 2, RH = 3 };

  AnymalCrocKinematics() = default;

  ~AnymalCrocKinematics() = default;

  AnymalCrocKinematics<SCALAR_T>* clone() const override;

  switched_model::vector3_s_t<SCALAR_T> positionBaseToFootInBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  joint_jacobian_t baseToFootJacobianInBaseFrame(size_t footIndex,
                                                 const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;

  switched_model::matrix3_s_t<SCALAR_T> footOrientationRelativeToBaseFrame(
      size_t footIndex, const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const override;
};

}  // namespace tpl

using AnymalCrocKinematics = tpl::AnymalCrocKinematics<double>;
using AnymalCrocKinematicsAd = tpl::AnymalCrocKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::AnymalCrocKinematics<double>;
extern template class anymal::tpl::AnymalCrocKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;
