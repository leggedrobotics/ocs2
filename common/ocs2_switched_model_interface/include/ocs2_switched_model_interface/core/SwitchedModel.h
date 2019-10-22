#pragma once

#include <Eigen/Dense>
#include <array>

namespace switched_model {

constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 12;
constexpr size_t GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORDINATE_SIZE;
constexpr size_t STATE_DIM = 24;
constexpr size_t INPUT_DIM = 24;

enum class FeetEnum { LF, RF, LH, RH };
using contact_flag_t = std::array<bool, NUM_CONTACT_POINTS>;

using base_coordinate_t = Eigen::Matrix<double, BASE_COORDINATE_SIZE, 1>;
using joint_coordinate_t = Eigen::Matrix<double, JOINT_COORDINATE_SIZE, 1>;
using generalized_coordinate_t = Eigen::Matrix<double, GENERALIZED_COORDINATE_SIZE, 1>;

}  // end of namespace switched_model


