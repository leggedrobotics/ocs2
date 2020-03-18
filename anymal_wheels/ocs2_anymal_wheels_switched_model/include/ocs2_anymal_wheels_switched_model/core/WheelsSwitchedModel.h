#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <Eigen/Dense>

namespace anymal {
template <typename SCALAR_T>
using extended_joint_coordinate_s_t = Eigen::Matrix<SCALAR_T, switched_model::JOINT_COORDINATE_SIZE + 4, 1>;
using extended_joint_coordinate_t = Eigen::Matrix<double, switched_model::JOINT_COORDINATE_SIZE + 4, 1>;
using extended_joint_coordinate_ad_t =
    Eigen::Matrix<ocs2::CppAdInterface<double>::ad_scalar_t, switched_model::JOINT_COORDINATE_SIZE + 4, 1>;

template <typename SCALAR_T>
inline extended_joint_coordinate_s_t<SCALAR_T> getExtendedJointCoordinates(
    const switched_model::joint_coordinate_s_t<SCALAR_T>& jointPositions) {
  extended_joint_coordinate_s_t<SCALAR_T> extendedJointCoordinate;
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

}  // namespace anymal
