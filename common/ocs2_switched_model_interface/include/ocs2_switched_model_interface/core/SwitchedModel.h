#pragma once

#include <Eigen/Dense>
#include <array>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace switched_model {

constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 12;
constexpr size_t GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;
constexpr size_t STATE_DIM = 24;
constexpr size_t INPUT_DIM = 24;

enum class FeetEnum { LF, RF, LH, RH };
using contact_flag_t = std::array<bool, NUM_CONTACT_POINTS>;

template<typename scalar_t>
using base_coordinate_s_t = Eigen::Matrix<scalar_t, BASE_COORDINATE_SIZE, 1>;
using base_coordinate_t = base_coordinate_s_t<double>;
using base_coordinate_ad_t = base_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template<typename scalar_t>
using joint_coordinate_s_t = Eigen::Matrix<scalar_t, JOINT_COORDINATE_SIZE, 1>;
using joint_coordinate_t = joint_coordinate_s_t<double>;
using joint_coordinate_ad_t = joint_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template<typename scalar_t>
using generalized_coordinate_s_t = Eigen::Matrix<scalar_t, GENERALIZED_COORDINATE_SIZE, 1>;
using generalized_coordinate_t = generalized_coordinate_s_t<double>;
using generalized_coordinate_ad_t = generalized_coordinate_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template<typename scalar_t>
using comkino_state_s_t = Eigen::Matrix<scalar_t, 2*BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE, 1>;
using comkino_state_t = comkino_state_s_t<double>;
using comkino_state_ad_t = comkino_state_s_t<ocs2::CppAdInterface<double>::ad_scalar_t>;

template<typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointPositions(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template segment<JOINT_COORDINATE_SIZE>(2*BASE_COORDINATE_SIZE);
}

template<typename scalar_t>
base_coordinate_s_t<scalar_t> getComPose(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template head<BASE_COORDINATE_SIZE>();
}

template<typename scalar_t>
base_coordinate_s_t<scalar_t> getComLocalVelocities(const comkino_state_s_t<scalar_t>& comkinoState) {
  return comkinoState.template segment<BASE_COORDINATE_SIZE>(BASE_COORDINATE_SIZE);
}

}  // end of namespace switched_model


