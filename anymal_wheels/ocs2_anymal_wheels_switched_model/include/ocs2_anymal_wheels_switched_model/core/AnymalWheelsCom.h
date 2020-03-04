/*
 * AnymalWheelsCom.h
 *
 *  Created on: Nov 25, 2019
 *      Author: Marko Bjelonic
 */

#pragma once

#include <ocs2_switched_model_interface/core/ComModelBase.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalWheelsCom : public switched_model::ComModelBase<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Constructor needed for initialization
   */
  AnymalWheelsCom();

  /**
   * Default destructor
   */
  ~AnymalWheelsCom() = default;

  AnymalWheelsCom<SCALAR_T>* clone() const override;

  void setJointConfiguration(const switched_model::joint_coordinate_s_t<SCALAR_T>& q) override;

  switched_model::vector3_s_t<SCALAR_T> comPositionBaseFrame() const override { return comPositionBaseFrame_; }

  SCALAR_T totalMass() const override { return totalMass_; }

  Eigen::Matrix<SCALAR_T, 6, 6> comInertia() const override { return comInertia_; }

 private:
  // cached values for current default joint configuration
  switched_model::vector3_s_t<SCALAR_T> comPositionBaseFrame_;
  switched_model::matrix6_s_t<SCALAR_T> comInertia_;
  SCALAR_T totalMass_;

  // Includes wheel joints
  using extended_joint_coordinate_t = Eigen::Matrix<SCALAR_T, switched_model::JOINT_COORDINATE_SIZE + 4, 1>;

  extended_joint_coordinate_t getExtendedJointCoordinates(const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) const {
    extended_joint_coordinate_t extendedJointCoordinate;
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
};

}  // namespace tpl

using AnymalWheelsCom = tpl::AnymalWheelsCom<double>;
using AnymalWheelsComAd = tpl::AnymalWheelsCom<ocs2::CppAdInterface<double>::ad_scalar_t>;

}  // namespace anymal

/**
 *  Explicit instantiation, for instantiation additional types, include the implementation file instead of this one.
 */
extern template class anymal::tpl::AnymalWheelsCom<double>;
extern template class anymal::tpl::AnymalWheelsCom<ocs2::CppAdInterface<double>::ad_scalar_t>;
