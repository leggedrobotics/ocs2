/*!
* @file    TestAnymalSwitchedModel.h
* @author  Jan Carius
* @date    Nov, 2017
*/

#pragma once

// model
#include <ocs2_anymal_bear_switched_model/core/AnymalBearCom.h>
#include <ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h>
#include <ocs2_anymal_bear_switched_model/generated/declarations.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <functional>
#include <random>

namespace anymal {

class TestAnymalSwitchedModel {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using scalar_t                 = double;
  using DIMS                     = ocs2::Dimensions<switched_model::STATE_DIM, switched_model::INPUT_DIM>;
  using JointIdentifiers         = iit::ANYmal::JointIdentifiers;
  using FeetEnum                 = switched_model::FeetEnum;
  using state_vector_t           = DIMS::state_vector_t;
  using state_matrix_t           = DIMS::state_matrix_t;
  using joint_coordinate_t       = switched_model::joint_coordinate_t;
  using base_coordinate_t        = switched_model::base_coordinate_t;
  using generalized_coordinate_t = switched_model::generalized_coordinate_t;
  using matrix3_t = switched_model::matrix3_t;
  using vector3_t = switched_model::vector3_t;

  TestAnymalSwitchedModel()
      : stanceLegs_({{true, true, true, true}}),
        posDist_{-20, 20},
        angleDist_{0.07, M_PI - 0.1},
        randAngle{std::bind(std::ref(angleDist_), std::ref(generator_))},
        randPos{std::bind(std::ref(posDist_), std::ref(generator_))} {}

  void init() {
    // nothing to do yet
  }

public:
  AnymalBearKinematics kinematics_;
  AnymalBearCom comDynamics_;
  AnymalBearComAd anymalComAd_;
  AnymalBearKinematicsAd anymalKinematicsAd_;

  switched_model::contact_flag_t stanceLegs_;
  std::function<bool(const Eigen::MatrixXd  &lhs, const Eigen::MatrixXd rhs)> matrixEquality_ = [](const Eigen::MatrixXd  &lhs, const Eigen::MatrixXd rhs) {return lhs.isApprox(rhs);};
  std::seed_seq seed_{4, 7, 93, 8};  // WOW! SO RANDOM!
  std::default_random_engine generator_{seed_};
  std::uniform_real_distribution<scalar_t> posDist_;    //{-20, 20};
  std::uniform_real_distribution<scalar_t> angleDist_;  //{0.07, M_PI-0.1};
  decltype(std::bind(std::ref(angleDist_), std::ref(generator_))) randAngle;
  decltype(std::bind(std::ref(posDist_), std::ref(generator_))) randPos;
};

} // namespace anymal
