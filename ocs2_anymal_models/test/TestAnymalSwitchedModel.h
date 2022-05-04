//
// Created by rgrandia on 25.09.20.
//

#pragma once

#include <gtest/gtest.h>

#include <functional>
#include <random>

#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h>
#include <ocs2_switched_model_interface/constraint/FootNormalConstraint.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/cost/MotionTrackingCost.h>
#include <ocs2_switched_model_interface/test/TestEvaluateConstraints.h>

namespace switched_model {

class TestAnymalSwitchedModel : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum JointIdentifiers { LF_HAA = 0, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE };

  TestAnymalSwitchedModel(std::unique_ptr<KinematicsModelBase<ocs2::scalar_t>> kinematics,
                          std::unique_ptr<KinematicsModelBase<ocs2::ad_scalar_t>> kinematicsAd,
                          std::unique_ptr<ComModelBase<ocs2::scalar_t>> comModel,
                          std::unique_ptr<ComModelBase<ocs2::ad_scalar_t>> comModelAd)
      : kinematics_(std::move(kinematics)),
        kinematicsAd_(std::move(kinematicsAd)),
        comModel_(std::move(comModel)),
        comModelAd_(std::move(comModelAd)),
        randAngle{std::bind(std::ref(angleDist_), std::ref(generator_))},
        randPos{std::bind(std::ref(posDist_), std::ref(generator_))} {}

  void testCosts() {
    // TODO
  }

  void testConstraints() {
    // TODO
  }

  void printKinematics() {
    const joint_coordinate_t qJoints = joint_coordinate_t::Zero();
    std::cout << "Joint coordinates:\n" << qJoints.transpose() << std::endl;
    std::cout << "Foot position LF:\n" << kinematics_->positionBaseToFootInBaseFrame(0, qJoints) << std::endl;
    std::cout << "Foot position RF:\n" << kinematics_->positionBaseToFootInBaseFrame(1, qJoints) << std::endl;
    std::cout << "Foot position LH:\n" << kinematics_->positionBaseToFootInBaseFrame(2, qJoints) << std::endl;
    std::cout << "Foot position RH:\n" << kinematics_->positionBaseToFootInBaseFrame(3, qJoints) << std::endl;
    std::cout << "Foot jacobian LF:\n" << kinematics_->baseToFootJacobianInBaseFrame(0, qJoints) << std::endl;
    std::cout << "Foot jacobian RF:\n" << kinematics_->baseToFootJacobianInBaseFrame(1, qJoints) << std::endl;
    std::cout << "Foot jacobian LH:\n" << kinematics_->baseToFootJacobianInBaseFrame(2, qJoints) << std::endl;
    std::cout << "Foot jacobian RH:\n" << kinematics_->baseToFootJacobianInBaseFrame(3, qJoints) << std::endl;
  }

  void testEndeffectorOrientation() {
    const base_coordinate_t basePose = base_coordinate_t::Zero();
    const joint_coordinate_t qJoints = joint_coordinate_t::Zero();
    matrix3_t o_R_b = rotationMatrixBaseToOrigin(getOrientation(basePose));
    std::cout << "Base orientation:\n" << o_R_b << std::endl;
    for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
      const matrix3_t o_R_f = kinematics_->footOrientationInOriginFrame(footIdx, basePose, qJoints);
      std::cout << "Foot orientation " << feetNames[footIdx] << ":\n" << o_R_f << std::endl;
      EXPECT_PRED2(matrixEquality_, o_R_f, o_R_b);
    }
  }

 protected:
  std::unique_ptr<KinematicsModelBase<ocs2::scalar_t>> kinematics_;
  std::unique_ptr<KinematicsModelBase<ocs2::ad_scalar_t>> kinematicsAd_;
  std::unique_ptr<ComModelBase<ocs2::scalar_t>> comModel_;
  std::unique_ptr<ComModelBase<ocs2::ad_scalar_t>> comModelAd_;

 private:
  contact_flag_t stanceLegs_ = {{true, true, true, true}};

  std::function<bool(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd rhs)> matrixEquality_ =
      [](const Eigen::MatrixXd& lhs, const Eigen::MatrixXd rhs) { return lhs.isApprox(rhs); };

  std::seed_seq seed_{4, 7, 93, 8};
  std::default_random_engine generator_{seed_};
  std::uniform_real_distribution<ocs2::scalar_t> posDist_{-20, 20};
  std::uniform_real_distribution<ocs2::scalar_t> angleDist_{0.07, M_PI - 0.1};
  decltype(std::bind(std::ref(angleDist_), std::ref(generator_))) randAngle;
  decltype(std::bind(std::ref(posDist_), std::ref(generator_))) randPos;
};

}  // namespace switched_model
