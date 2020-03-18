/*!
 * @file    TestAnymalSwitchedModel.h
 * @author  Jan Carius
 * @date    Nov, 2017
 */

#pragma once

// model
#include <ocs2_anymal_croc_switched_model/core/AnymalCrocCom.h>
#include <ocs2_anymal_croc_switched_model/core/AnymalCrocKinematics.h>
#include <ocs2_anymal_croc_switched_model/generated/declarations.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <functional>
#include <random>

namespace anymal {

class TestAnymalSwitchedModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t = double;
  using DIMS = ocs2::Dimensions<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
  using JointIdentifiers = iit::ANYmal::JointIdentifiers;
  using FeetEnum = switched_model::FeetEnum;
  using state_vector_t = DIMS::state_vector_t;
  using state_matrix_t = DIMS::state_matrix_t;
  using joint_coordinate_t = switched_model::joint_coordinate_t;
  using base_coordinate_t = switched_model::base_coordinate_t;
  using generalized_coordinate_t = switched_model::generalized_coordinate_t;
  using matrix3_t = switched_model::matrix3_t;
  using vector3_t = switched_model::vector3_t;

  TestAnymalSwitchedModel() : stanceLegs_({{true, true, true, true}}) {}

  void init() {
    // nothing to do yet
  }

 public:
  AnymalCrocKinematics kinematics_;
  AnymalCrocCom comDynamics_;
  AnymalCrocComAd anymalComAd_;
  AnymalCrocKinematicsAd anymalKinematicsAd_;

  std::array<bool, 4> stanceLegs_;
  std::function<bool(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd rhs)> matrixEquality_ =
      [](const Eigen::MatrixXd& lhs, const Eigen::MatrixXd rhs) { return lhs.isApprox(rhs); };
};

}  // namespace anymal
