#pragma once

#include <Eigen/Dense>
#include <array>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace switched_model {
/**
 * Switched model definition:
 *
 * state = [theta, p, w, v, q (4x)]
 * theta: EulerXYZ (3x1)
 * p: CoM position in Origin frame (3x1)
 * w: CoM linear velocity in Base Frame (3x1)
 * v: CoM angular velocity in Base Frame (3x1)
 * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
 *
 * input = [lambda (4x), qj (4x)]
 * lambda: Force at the EE [LF, RF, LH, RH] in Base Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
 */

/**
 * Rbd state definition:
 * !! notice the definition is w.r.t base center, not CoM !!
 *
 * [ theta, p, q (4x), w, v, qj  (4x)]
 * theta: EulerXYZ (3x1)
 * p: Base position in Origin frame (3x1)
 * q: Joint angles per leg [HAA, HFE, KFE] (3x1) [4x]
 * w: Base linear velocity in Base Frame (3x1)
 * v: Base angular velocity in Base Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1) [4x]
 */

constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 12;
constexpr size_t GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;  // 18
constexpr size_t RBD_STATE_DIM = 2 * GENERALIZED_COORDINATE_SIZE;                             // 36
constexpr size_t STATE_DIM = 2 * BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;                // 24
constexpr size_t INPUT_DIM = 3 * NUM_CONTACT_POINTS + JOINT_COORDINATE_SIZE;                  // 24

/* Import ocs2 types into the switched_model namespace */
using ocs2::scalar_array_t;
using ocs2::scalar_t;

/* Feet related declarations */
enum class FeetEnum { LF, RF, LH, RH };
template <typename T>
using feet_array_t = std::array<T, NUM_CONTACT_POINTS>;  // Fixed size container fer foot
const feet_array_t<std::string> feetNames{"LF", "RF", "LH", "RH"};
using contact_flag_t = feet_array_t<bool>;  // Contact state per foot, true = in contact, false = not in contact

template <typename SCALAR_T>
using vector3_s_t = Eigen::Matrix<SCALAR_T, 3, 1>;
using vector3_t = vector3_s_t<scalar_t>;
using vector3_ad_t = vector3_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using matrix3_s_t = Eigen::Matrix<SCALAR_T, 3, 3>;
using matrix3_t = matrix3_s_t<scalar_t>;
using matrix3_ad_t = matrix3_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using vector6_s_t = Eigen::Matrix<SCALAR_T, 6, 1>;
using vector6_t = vector6_s_t<scalar_t>;
using vector6_ad_t = vector6_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using matrix6_s_t = Eigen::Matrix<SCALAR_T, 6, 6>;
using matrix6_t = matrix6_s_t<scalar_t>;
using matrix6_ad_t = matrix6_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using base_coordinate_s_t = Eigen::Matrix<SCALAR_T, BASE_COORDINATE_SIZE, 1>;
using base_coordinate_t = base_coordinate_s_t<scalar_t>;
using base_coordinate_ad_t = base_coordinate_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using joint_coordinate_s_t = Eigen::Matrix<SCALAR_T, JOINT_COORDINATE_SIZE, 1>;
using joint_coordinate_t = joint_coordinate_s_t<scalar_t>;
using joint_coordinate_ad_t = joint_coordinate_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using generalized_coordinate_s_t = Eigen::Matrix<SCALAR_T, GENERALIZED_COORDINATE_SIZE, 1>;
using generalized_coordinate_t = generalized_coordinate_s_t<scalar_t>;
using generalized_coordinate_ad_t = generalized_coordinate_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using com_state_s_t = Eigen::Matrix<SCALAR_T, 2 * BASE_COORDINATE_SIZE, 1>;
using com_state_t = com_state_s_t<scalar_t>;
using com_state_ad_t = com_state_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using comkino_state_s_t = Eigen::Matrix<SCALAR_T, STATE_DIM, 1>;
using comkino_state_t = comkino_state_s_t<scalar_t>;
using comkino_state_ad_t = comkino_state_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using comkino_input_s_t = Eigen::Matrix<SCALAR_T, INPUT_DIM, 1>;
using comkino_input_t = comkino_input_s_t<scalar_t>;
using comkino_input_ad_t = comkino_input_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
using rbd_state_s_t = Eigen::Matrix<SCALAR_T, 2 * GENERALIZED_COORDINATE_SIZE, 1>;
using rbd_state_t = rbd_state_s_t<scalar_t>;
using rbd_state_ad_t = rbd_state_s_t<ocs2::CppAdInterface<scalar_t>::ad_scalar_t>;

template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> getComPose(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template head<BASE_COORDINATE_SIZE>();
}

template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> getBasePose(const generalized_coordinate_s_t<SCALAR_T>& generalizedCoordinate) {
  return generalizedCoordinate.template head<BASE_COORDINATE_SIZE>();
}

template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> getBasePose(const rbd_state_s_t<SCALAR_T>& rbdState) {
  return rbdState.template head<BASE_COORDINATE_SIZE>();
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getOrientation(const base_coordinate_s_t<SCALAR_T>& baseCoordinate) {
  return baseCoordinate.template head<3>();
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getPositionInOrigin(const base_coordinate_s_t<SCALAR_T>& baseCoordinate) {
  return baseCoordinate.template tail<3>();
}

template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> getBaseLocalVelocity(const rbd_state_s_t<SCALAR_T>& rbdState) {
  return rbdState.template segment<BASE_COORDINATE_SIZE>(GENERALIZED_COORDINATE_SIZE);
}

template <typename SCALAR_T>
base_coordinate_s_t<SCALAR_T> getComLocalVelocities(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getAngularVelocity(const base_coordinate_s_t<SCALAR_T>& baseTwist) {
  return baseTwist.template head<3>();
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getAngularAcceleration(const base_coordinate_s_t<SCALAR_T>& baseAcceleration) {
  return baseAcceleration.template head<3>();
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getLinearVelocity(const base_coordinate_s_t<SCALAR_T>& baseTwist) {
  return baseTwist.template tail<3>();
}

template <typename SCALAR_T>
vector3_s_t<SCALAR_T> getLinearAcceleration(const base_coordinate_s_t<SCALAR_T>& baseAcceleration) {
  return baseAcceleration.template tail<3>();
}

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> getJointPositions(const comkino_state_s_t<SCALAR_T>& comkinoState) {
  return comkinoState.template segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> getJointPositions(const generalized_coordinate_s_t<SCALAR_T>& generalizedCoordinate) {
  return generalizedCoordinate.template segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> getJointPositions(const rbd_state_s_t<SCALAR_T>& rbdState) {
  return rbdState.template segment<JOINT_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> getJointVelocities(const comkino_input_s_t<SCALAR_T>& comkinoInput) {
  return comkinoInput.template segment<JOINT_COORDINATE_SIZE>(NUM_CONTACT_POINTS * 3);
}

template <typename SCALAR_T>
joint_coordinate_s_t<SCALAR_T> getJointVelocities(const rbd_state_s_t<SCALAR_T>& rbdState) {
  return rbdState.template segment<JOINT_COORDINATE_SIZE>(GENERALIZED_COORDINATE_SIZE + BASE_COORDINATE_SIZE);
}

template <typename SCALAR_T>
feet_array_t<vector3_s_t<SCALAR_T>> toArray(const joint_coordinate_s_t<SCALAR_T>& valuesAsVector) {
  return {valuesAsVector.template segment<3>(0), valuesAsVector.template segment<3>(3), valuesAsVector.template segment<3>(6),
          valuesAsVector.template segment<3>(9)};
}

}  // end of namespace switched_model
