//
// Created by rgrandia on 25.09.20.
//

#pragma once

#include <gtest/gtest.h>

#include <functional>
#include <random>

#include <ocs2_switched_model_interface/Dimensions.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h>
#include <ocs2_switched_model_interface/constraint/FootNormalConstraint.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/WholebodyDynamics.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/test/TestEvaluateConstraints.h>

namespace switched_model {

class TestAnymalSwitchedModel : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum JointIdentifiers { LF_HAA = 0, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE, LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE };

  TestAnymalSwitchedModel(std::unique_ptr<KinematicsModelBase<ocs2::scalar_t>> kinematics,
                          std::unique_ptr<KinematicsModelBase<ocs2::ad_scalar_t>> kinematicsAd,
                          std::unique_ptr<ComModelBase<ocs2::scalar_t>> comModel,
                          std::unique_ptr<ComModelBase<ocs2::ad_scalar_t>> comModelAd,
                          std::unique_ptr<WholebodyDynamics<ocs2::scalar_t>> wholebodyDynamics)
      : kinematics_(std::move(kinematics)),
        kinematicsAd_(std::move(kinematicsAd)),
        comModel_(std::move(comModel)),
        comModelAd_(std::move(comModelAd)),
        wholebodyDynamics_(std::move(wholebodyDynamics)),
        randAngle{std::bind(std::ref(angleDist_), std::ref(generator_))},
        randPos{std::bind(std::ref(posDist_), std::ref(generator_))} {}

  void testCosts() {
    SwitchedModelModeScheduleManager modeScheduleManager(nullptr, nullptr, nullptr);
    MotionTrackingCost::Weights weights;
    weights.eulerXYZ << 100.0, 200.0, 200.0;
    weights.comPosition << 1000.0, 1000.0, 1500.0;
    weights.comAngularVelocity << 5.0, 10.0, 10.0;
    weights.comLinearVelocity << 15.0, 15.0, 30.0;
    weights.jointPosition.setConstant(1.0);  // some regularization
    weights.contactForce.setConstant(0.001);
    weights.footPosition.setConstant(60.0);
    weights.footVelocity.setConstant(1.0);

    switched_model::MotionTrackingCost motionTrackingCost(weights, modeScheduleManager, *kinematics_, *kinematicsAd_, *comModel_, true);
    ocs2::scalar_t t = 0.0;
    comkino_state_t x = comkino_state_t::Zero();
    x.segment<JOINT_COORDINATE_SIZE>(2 * BASE_COORDINATE_SIZE) << -0.25, 0.60, -0.85, 0.25, 0.60, -0.85, -0.25, -0.60, 0.85, 0.25, -0.60,
        0.85;
    comkino_input_t u = weightCompensatingInputs(*comModel_, constantFeetArray(true), x.head(3));
    ocs2::CostDesiredTrajectories costDesiredTrajectories({t}, {x}, {u});

    const auto approx = motionTrackingCost.getQuadraticApproximation(t, x, u, costDesiredTrajectories);

    // Reference equals current state input, should have zero cost and zero gradient.
    ASSERT_LT(approx.f, 1e-9);
    ASSERT_LT(approx.dfdx.norm(), 1e-9);
    ASSERT_LT(approx.dfdu.norm(), 1e-9);
    ASSERT_TRUE(approx.dfdxx.allFinite());
    ASSERT_TRUE(approx.dfduu.allFinite());
    ASSERT_TRUE(approx.dfdux.allFinite());
  }

  void testConstraints() {
    EndEffectorConstraintSettings endEffectorConstraintSettings(2, 3);
    endEffectorConstraintSettings.A << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    endEffectorConstraintSettings.b << 2.0, 3.0;
    EXPECT_NO_THROW(evaluateConstraint<EndEffectorVelocityConstraint>(*comModelAd_, *kinematicsAd_, endEffectorConstraintSettings));
    EXPECT_NO_THROW(
        evaluateConstraint<EndEffectorVelocityInFootFrameConstraint>(*comModelAd_, *kinematicsAd_, endEffectorConstraintSettings));

    FootNormalConstraintMatrix footNormalConstraintMatrix;
    footNormalConstraintMatrix.positionMatrix << 0.1, 0.2, 0.3;
    footNormalConstraintMatrix.velocityMatrix << 0.4, 0.5, 0.6;
    footNormalConstraintMatrix.constant = 2.0;
    EXPECT_NO_THROW(evaluateConstraint<FootNormalConstraint>(*comModelAd_, *kinematicsAd_, footNormalConstraintMatrix));
  }

  void testBaseDynamics() {
    // Check if the base accelerations computed with full dynamics is the same as the one in comModel, which uses only the top 6 rows.
    if (wholebodyDynamics_) {
      for (int i = 0; i < 100; i++) {
        const switched_model::rbd_state_t rbdState = switched_model::rbd_state_t::Random();
        const switched_model::generalized_coordinate_t externalForces = switched_model::generalized_coordinate_t::Random();
        const switched_model::base_coordinate_t forcesOnBaseInBaseFrame = externalForces.head<6>();
        const switched_model::joint_coordinate_t tau = switched_model::joint_coordinate_t::Random();
        const switched_model::base_coordinate_t qBase = switched_model::getBasePose(rbdState);
        const switched_model::base_coordinate_t qdBase = switched_model::getBaseLocalVelocity(rbdState);
        const switched_model::joint_coordinate_t qj = switched_model::getJointPositions(rbdState);
        const switched_model::joint_coordinate_t dqj = switched_model::getJointVelocities(rbdState);

        // Compute with full dynamics
        const auto fullRbdDynamics = wholebodyDynamics_->getDynamicsTerms(rbdState);
        switched_model::generalized_coordinate_t dynamicsForce = externalForces - fullRbdDynamics.G - fullRbdDynamics.C;
        dynamicsForce.tail<JOINT_COORDINATE_SIZE>() += tau;
        const switched_model::generalized_coordinate_t generalizedAcceleration = fullRbdDynamics.M.ldlt().solve(dynamicsForce);

        // Compute with base dynamics, use joint accelerations as given by full forward dynamics.
        const switched_model::joint_coordinate_t ddqj = generalizedAcceleration.tail<JOINT_COORDINATE_SIZE>();
        const auto baseAcceleration =
            this->comModel_->calculateBaseLocalAccelerations(qBase, qdBase, qj, dqj, ddqj, forcesOnBaseInBaseFrame);

        ASSERT_TRUE(baseAcceleration.isApprox(generalizedAcceleration.head<6>()));
      }
    }
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

  void printComModel() {
    std::cout << "comPositionBaseFrameDefault:\n" << comModel_->comPositionBaseFrame() << std::endl;
    std::cout << "comInertiaDefault:\n" << comModel_->comInertia() << std::endl;
    const joint_coordinate_t qJoints = joint_coordinate_t::Zero();
    std::cout << "Joint coordinates:\n" << qJoints.transpose() << std::endl;
    comModel_->setJointConfiguration(qJoints);
    std::cout << "comPositionBaseFrame:\n" << comModel_->comPositionBaseFrame() << std::endl;
    std::cout << "comInertia:\n" << comModel_->comInertia() << std::endl;
  };

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

  void testEndeffectorAlignedYAxisRandomHFEKFE() {
    /** Y Axis coincidence
     *
     * With only the HFE, and KFE joints rotated, the y-axis of the foot should still be aligned with the
     * y-axis of the base, at any base pose.
     */
    generalized_coordinate_t x;
    x.setZero();
    x[0] = randAngle();
    x[1] = randAngle();
    x[2] = randAngle();
    x[3] = randPos();
    x[4] = randPos();
    x[5] = randPos();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RF_HFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RF_KFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LF_HFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LF_KFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RH_HFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RH_KFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LH_HFE] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LH_KFE] = randAngle();
    joint_coordinate_t qJoints = getJointPositions(x);

    const base_coordinate_t basePose = getBasePose(x);
    matrix3_t o_R_b = rotationMatrixBaseToOrigin(getOrientation(basePose));
    vector3_t yAxis;
    yAxis << 0, 1, 0;
    std::cout << "Base orientation:\n" << o_R_b << std::endl;
    std::cout << "Axis:" << yAxis.transpose() << std::endl;
    for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
      const matrix3_t o_R_f = kinematics_->footOrientationInOriginFrame(footIdx, basePose, qJoints);
      std::cout << "Foot orientation " << feetNames[footIdx] << ":\n" << o_R_f << std::endl;
      EXPECT_PRED2(matrixEquality_, o_R_f * yAxis, o_R_b * yAxis);
    }
  }

  void testEndeffectorAlignedXAxisRandomHAA() {
    /** X Axis coincidence
     *
     * With only the HAA joints rotated, the x-axis of the foot should still be aligned with the
     * x-axis of the base, at any base pose.
     */

    generalized_coordinate_t x;
    x.setZero();
    x[0] = randAngle();
    x[1] = randAngle();
    x[2] = randAngle();
    x[3] = randPos();
    x[4] = randPos();
    x[5] = randPos();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RF_HAA] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LF_HAA] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::RH_HAA] = randAngle();
    x[BASE_COORDINATE_SIZE + JointIdentifiers::LH_HAA] = randAngle();
    joint_coordinate_t qJoints = getJointPositions(x);

    const base_coordinate_t basePose = getBasePose(x);
    matrix3_t o_R_b = rotationMatrixBaseToOrigin(getOrientation(basePose));
    vector3_t xAxis;
    xAxis << 1, 0, 0;
    std::cout << "Base orientation:\n" << o_R_b << std::endl;
    std::cout << "Axis:" << xAxis.transpose() << std::endl;
    for (int footIdx = 0; footIdx < NUM_CONTACT_POINTS; footIdx++) {
      const matrix3_t o_R_f = kinematics_->footOrientationInOriginFrame(footIdx, basePose, qJoints);
      std::cout << "Foot orientation " << feetNames[footIdx] << ":\n" << o_R_f << std::endl;
      EXPECT_PRED2(matrixEquality_, o_R_f * xAxis, o_R_b * xAxis);
    }
  }

 protected:
  std::unique_ptr<KinematicsModelBase<ocs2::scalar_t>> kinematics_;
  std::unique_ptr<KinematicsModelBase<ocs2::ad_scalar_t>> kinematicsAd_;
  std::unique_ptr<ComModelBase<ocs2::scalar_t>> comModel_;
  std::unique_ptr<ComModelBase<ocs2::ad_scalar_t>> comModelAd_;
  std::unique_ptr<WholebodyDynamics<ocs2::scalar_t>> wholebodyDynamics_;

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
