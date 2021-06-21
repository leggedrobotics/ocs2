#pragma once

// ocs2
#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <Eigen/Dense>
#include <array>
#include <cstddef>

// ros
#include <ros/package.h>

/**
 * centroidal model definition:
 *
 * state = [linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_angles]
 * input = [feet_force, arm_ee_wrench, joint_velocities] + slack variables corresponding to the inequality constraints
 * lambda: Force order [LF, RF, LH, RH, arm_ee_force, arm_ee_torque] in World Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1) + Arm joint velocities
 */

namespace ocs2 {
namespace legged_robot {
const std::string ROBOT_NAME_ = "legged_robot";
const std::string ROBOT_URDF_PATH_ = ros::package::getPath("ocs2_legged_robot_example") + "/urdf/" + ROBOT_NAME_ + ".urdf";
const std::string ROBOT_COMMAND_PATH_ = ros::package::getPath("ocs2_legged_robot_example") + "/config/command/" + "targetCommand.info";

constexpr size_t DOF_PER_LEG_NUM_ = 3;
constexpr size_t FOOT_CONTACTS_NUM_ = 4;  // each contact involves a 3-DOF force
constexpr size_t FOOT_CONTACTS_DOF_ = 3;

constexpr size_t TOTAL_CONTACTS_DIM_ = FOOT_CONTACTS_DOF_ * FOOT_CONTACTS_NUM_;
constexpr size_t BASE_DOF_NUM_ = 6;
constexpr size_t ACTUATED_DOF_NUM_ = DOF_PER_LEG_NUM_ * FOOT_CONTACTS_NUM_;
constexpr size_t GENERALIZED_VEL_NUM_ = BASE_DOF_NUM_ + ACTUATED_DOF_NUM_;

constexpr size_t STATE_DIM_ = 2 * BASE_DOF_NUM_ + ACTUATED_DOF_NUM_;
constexpr size_t INPUT_DIM_ = TOTAL_CONTACTS_DIM_ + ACTUATED_DOF_NUM_;

// Used for task-space cost: [quat_x, quat_y, quat_z, quat_w, p_x, p_y, p_z]
constexpr size_t EE_COST_REFERENCE_DIM_ = 7;

// An approximation of the total mass, to be used when defining the nominal input force_z
const double ROBOT_TOTAL_MASS_ = 50;

// Names of feet contacts come first
const static std::array<std::string, FOOT_CONTACTS_NUM_> CONTACT_POINTS_NAMES_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

// This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
const static std::array<std::string, 3 * FOOT_CONTACTS_NUM_ + 6> JOINT_NAMES_ = {
    "LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE", "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

const static std::vector<std::string> LEGGED_ROBOT_3_DOF_CONTACT_NAMES_ = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
const static std::vector<std::string> LEGGED_ROBOT_6_DOF_CONTACT_NAMES_ = {};

template <typename T>
using feet_array_t = std::array<T, FOOT_CONTACTS_NUM_>;
using contact_flag_t = feet_array_t<bool>;

/* Import ocs2 types into the alma_c namespace */
using ocs2::matrix_array_t;
using ocs2::matrix_t;
using ocs2::scalar_array_t;
using ocs2::scalar_t;
using ocs2::size_array_t;
using ocs2::vector_array_t;
using ocs2::vector_t;

using ocs2::ScalarFunctionQuadraticApproximation;
using ocs2::VectorFunctionLinearApproximation;
using ocs2::VectorFunctionQuadraticApproximation;

/* Define fixed-size types */
using state_vector_t = Eigen::Matrix<scalar_t, STATE_DIM_, 1>;
using input_vector_t = Eigen::Matrix<scalar_t, INPUT_DIM_, 1>;

using state_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM_, STATE_DIM_>;
using input_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM_, INPUT_DIM_>;
using input_state_matrix_t = Eigen::Matrix<scalar_t, INPUT_DIM_, STATE_DIM_>;
using state_input_matrix_t = Eigen::Matrix<scalar_t, STATE_DIM_, INPUT_DIM_>;

using state_vector_array_t = std::vector<state_vector_t, Eigen::aligned_allocator<state_vector_t>>;
using input_vector_array_t = std::vector<input_vector_t, Eigen::aligned_allocator<input_vector_t>>;
using state_matrix_array_t = std::vector<state_matrix_t, Eigen::aligned_allocator<state_matrix_t>>;
using input_matrix_array_t = std::vector<input_matrix_t, Eigen::aligned_allocator<input_matrix_t>>;
using input_state_matrix_array_t = std::vector<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>>;

template <typename scalar_t>
using vector3_s_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector3_t = vector3_s_t<double>;
using vector3_ad_t = vector3_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using matrix3_s_t = Eigen::Matrix<scalar_t, 3, 3>;
using matrix3_t = matrix3_s_t<double>;
using matrix3_ad_t = matrix3_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using vector6_s_t = Eigen::Matrix<scalar_t, 6, 1>;
using vector6_t = vector6_s_t<double>;
using vector6_ad_t = vector6_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using matrix6_s_t = Eigen::Matrix<scalar_t, 6, 6>;
using matrix6_t = matrix6_s_t<double>;
using matrix6_ad_t = matrix6_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using base_coordinate_s_t = Eigen::Matrix<scalar_t, BASE_DOF_NUM_, 1>;
using base_coordinate_t = base_coordinate_s_t<double>;
using base_coordinate_ad_t = base_coordinate_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using joint_coordinate_s_t = Eigen::Matrix<scalar_t, ACTUATED_DOF_NUM_, 1>;
using joint_coordinate_t = joint_coordinate_s_t<double>;
using joint_coordinate_ad_t = joint_coordinate_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using generalized_coordinate_s_t = Eigen::Matrix<scalar_t, GENERALIZED_VEL_NUM_, 1>;
using generalized_coordinate_t = generalized_coordinate_s_t<double>;
using generalized_coordinate_ad_t = generalized_coordinate_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using state_s_t = Eigen::Matrix<scalar_t, STATE_DIM_, 1>;
using state_t = state_s_t<double>;
using state_ad_t = state_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using input_s_t = Eigen::Matrix<scalar_t, INPUT_DIM_, 1>;
using input_t = input_s_t<double>;
using input_ad_t = input_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using rbd_joint_coordinate_s_t = Eigen::Matrix<scalar_t, ACTUATED_DOF_NUM_, 1>;
using rbd_joint_coordinate_t = rbd_joint_coordinate_s_t<double>;
using rbd_joint_coordinate_ad_t = rbd_joint_coordinate_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
using rbd_state_s_t = Eigen::Matrix<scalar_t, 2 * GENERALIZED_VEL_NUM_, 1>;
using rbd_state_t = rbd_state_s_t<double>;
using rbd_state_ad_t = rbd_state_s_t<ocs2::CppAdInterface::ad_scalar_t>;

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getComPose(const state_s_t<scalar_t>& state) {
  return state.template segment<BASE_DOF_NUM_>(6);
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBasePose(const generalized_coordinate_s_t<scalar_t>& generalizedCoordinate) {
  return generalizedCoordinate.template head<BASE_DOF_NUM_>();
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBasePose(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template head<BASE_DOF_NUM_>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getOrientation(base_coordinate_s_t<scalar_t> baseCoordinate) {
  return baseCoordinate.template tail<3>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getPositionInOrigin(base_coordinate_s_t<scalar_t> baseCoordinate) {
  return baseCoordinate.template head<3>();
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getBaseLocalVelocity(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<BASE_DOF_NUM_>(GENERALIZED_VEL_NUM_);
}

template <typename scalar_t>
vector3_s_t<scalar_t> getAngularVelocity(base_coordinate_s_t<scalar_t> baseTwist) {
  return baseTwist.template tail<3>();
}

template <typename scalar_t>
vector3_s_t<scalar_t> getLinearVelocity(base_coordinate_s_t<scalar_t> baseTwist) {
  return baseTwist.template head<3>();
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointPositions(const state_s_t<scalar_t>& state) {
  return state.template segment<ACTUATED_DOF_NUM_>(2 * BASE_DOF_NUM_);
}

template <typename scalar_t>
base_coordinate_s_t<scalar_t> getJointPositions(const generalized_coordinate_s_t<scalar_t>& generalizedCoordinate) {
  return generalizedCoordinate.template segment<ACTUATED_DOF_NUM_>(BASE_DOF_NUM_);
}

template <typename scalar_t>
rbd_joint_coordinate_s_t<scalar_t> getJointPositions(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<ACTUATED_DOF_NUM_>(BASE_DOF_NUM_);
}

template <typename scalar_t>
joint_coordinate_s_t<scalar_t> getJointVelocities(const input_s_t<scalar_t>& input) {
  return input.template segment<ACTUATED_DOF_NUM_>(TOTAL_CONTACTS_DIM_);
}

template <typename scalar_t>
rbd_joint_coordinate_s_t<scalar_t> getJointVelocities(const rbd_state_s_t<scalar_t>& rbdState) {
  return rbdState.template segment<ACTUATED_DOF_NUM_>(GENERALIZED_VEL_NUM_ + BASE_DOF_NUM_);
}

}  // namespace legged_robot
}  // namespace ocs2
