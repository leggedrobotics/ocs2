#pragma once

#include <Eigen/Dense>
#include <array>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace switched_model {
/**
 * Switched model definition:
 *
 * state = [theta, p, v, w, q (4x)]
 * theta: EulerXYZ (3x1)
 * p: CoM position in Origin frame (3x1)
 * v: CoM angular velocity in Base Frame (3x1)
 * w: CoM linear velocity in Base Frame (3x1)
 * q: Joint angles per leg [HAA, HFE, KFE] (3x1)
 *
 * input = [lambda (4x), qj (4x)]
 * lambda: Force at the EE [LF, RF, LH, RH] in Base Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
 */

constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 12;
constexpr size_t GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;  // 18
constexpr size_t RBD_STATE_DIM = 2 * GENERALIZED_COORDINATE_SIZE;                             // 36
constexpr size_t STATE_DIM = 2 * BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;                // 24
constexpr size_t INPUT_DIM = 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE;                  // 24

enum class FeetEnum : int { LF = 0, RF = 1, LH = 2, RH = 3 };
const std::array<std::string, NUM_CONTACT_POINTS> FeetNames{"LF", "RF", "LH", "RH"};

using contact_flag_t = std::array<bool, NUM_CONTACT_POINTS>;

template <typename scalar_t>
using vector3_s_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector3_t = vector3_s_t<double>;
using vector3_ad_t = vector3_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using matrix3_s_t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix3_t = matrix3_s_t<double>;
using matrix3_ad_t = matrix3_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using vector6_s_t = Eigen::Matrix<scalar_t, 6, 1>;
using vector6_t = vector6_s_t<double>;
using vector6_ad_t = vector6_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using matrix6_s_t = Eigen::Matrix<scalar_t, 6, 6>;
using matrix6_t = matrix6_s_t<double>;
using matrix6_ad_t = matrix6_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using base_coordinate_s_t = Eigen::Matrix<scalar_t, BASE_COORDINATE_SIZE, 1>;
using base_coordinate_t = base_coordinate_s_t<double>;
using base_coordinate_ad_t = base_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using joint_coordinate_s_t = Eigen::Matrix<scalar_t, JOINT_COORDINATE_SIZE, 1>;
using joint_coordinate_t = joint_coordinate_s_t<double>;
using joint_coordinate_ad_t = joint_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using generalized_coordinate_s_t = Eigen::Matrix<scalar_t, GENERALIZED_COORDINATE_SIZE, 1>;
using generalized_coordinate_t = generalized_coordinate_s_t<double>;
using generalized_coordinate_ad_t = generalized_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using com_state_s_t = Eigen::Matrix<scalar_t, 2 * BASE_COORDINATE_SIZE, 1>;
using com_state_t = com_state_s_t<double>;
using com_state_ad_t = com_state_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using comkino_state_s_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
using comkino_state_t = comkino_state_s_t<double>;
using comkino_state_ad_t = comkino_state_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using comkino_input_s_t = Eigen::Matrix<scalar_t, STATE_DIM, 1>;
using comkino_input_t = comkino_input_s_t<double>;
using comkino_input_ad_t = comkino_input_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
using rbd_state_s_t = Eigen::Matrix<scalar_t, 2 * GENERALIZED_COORDINATE_SIZE, 1>;
using rbd_state_t = rbd_state_s_t<double>;
using rbd_state_ad_t = rbd_state_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getComPose(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template head<BASE_COORDINATE_SIZE>();
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBasePose(const generalized_coordinate_s_t<scalar_t>& generalizedCoordinate) {
  return generalizedCoordinate.template head<BASE_COORDINATE_SIZE>();
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBasePose(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template head<BASE_COORDINATE_SIZE>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getOrientation(base_coordinate_s_t<scalar_t> baseCoordinate) {
  return baseCoordinate.template head<3>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getPositionInOrigin(base_coordinate_s_t<scalar_t> baseCoordinate) {
  return baseCoordinate.template tail<3>();
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBaseLocalVelocity(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<BASE_COORDINATE_SIZE>(GENERALIZED_COORDINATE_SIZE);
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getComLocalVelocities(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename scalar_t>
vector3_s_t<scalar_t> getAngularVelocity(base_coordinate_s_t<scalar_t> baseTwist) {
  return baseTwist.template head<3>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getLinearVelocity(base_coordinate_s_t<scalar_t> baseTwist) {
  return baseTwist.template tail<3>();
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointPositions(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getJointPositions(const generalized_coordinate_s_t<scalar_t>& generalizedCoordinate) {
  return generalizedCoordinate.template segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointPositions(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointVelocities(const comkino_input_s_t<scalar_t>& comkinoInput) {
  return comkinoInput.template segment<JOINT_COORDINATE_SIZE>(NUM_CONTACT_POINTS * 3);
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointVelocities(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<JOINT_COORDINATE_SIZE>(GENERALIZED_COORDINATE_SIZE + BASE_COORDINATE_SIZE);
}

}  // end of namespace switched_model
