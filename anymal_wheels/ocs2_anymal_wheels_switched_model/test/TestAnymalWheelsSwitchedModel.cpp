/*!
 * @file   TestAnymalWheelsSwitchedModel.cpp
 * @author Marko Bjelonic
 * @date   Nov 27, 2019
 */

#include <gtest/gtest.h>

#include "include/TestAnymalWheelsSwitchedModel.h"
#include <iit/rbd/traits/TraitSelector.h>

#include "ocs2_anymal_wheels_switched_model/generated/jacobians.h"
#include "ocs2_anymal_wheels_switched_model/generated/transforms.h"

using namespace anymal;

class SwitchedModelTests : public ::testing::Test, public TestAnymalWheelsSwitchedModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

TEST_F(SwitchedModelTests, Kinematics) {
  switched_model::joint_coordinate_t qJoints;
  qJoints.setZero();
  std::cout << "Joint coordinate:\n" << qJoints.transpose() << std::endl;

  Eigen::Vector3d footPosition_LF = kinematics_.positionBaseToFootInBaseFrame(0, qJoints);
  std::cout << "Foot position LF:\n" << footPosition_LF << std::endl;

  Eigen::Vector3d footPosition_RF = kinematics_.positionBaseToFootInBaseFrame(1, qJoints);
  std::cout << "Foot position RF:\n" << footPosition_RF << std::endl;

  Eigen::Vector3d footPosition_LH = kinematics_.positionBaseToFootInBaseFrame(2, qJoints);
  std::cout << "Foot position LH:\n" << footPosition_LH << std::endl;

  Eigen::Vector3d footPosition_RH = kinematics_.positionBaseToFootInBaseFrame(3, qJoints);
  std::cout << "Foot position RH:\n" << footPosition_RH << std::endl;

  Eigen::Matrix<double, 6, 12> footJacobian_LF = kinematics_.baseToFootJacobianInBaseFrame(0, qJoints);
  std::cout << "Foot jacobian LF:\n" << footJacobian_LF << std::endl;

  Eigen::Matrix<double, 6, 12> footJacobian_RF = kinematics_.baseToFootJacobianInBaseFrame(1, qJoints);
  std::cout << "Foot jacobian RF:\n" << footJacobian_RF << std::endl;

  Eigen::Matrix<double, 6, 12> footJacobian_LH = kinematics_.baseToFootJacobianInBaseFrame(2, qJoints);
  std::cout << "Foot jacobian LH:\n" << footJacobian_LH << std::endl;

  Eigen::Matrix<double, 6, 12> footJacobian_RH = kinematics_.baseToFootJacobianInBaseFrame(3, qJoints);
  std::cout << "Foot jacobian RH:\n" << footJacobian_RH << std::endl;
}

TEST_F(SwitchedModelTests, ComDynamics) {
  switched_model::joint_coordinate_t joint_cor;
  joint_cor.setZero();
  std::cout << "Joint coordinates:\n" << joint_cor.transpose() << std::endl;

  std::cout << "comPositionBaseFrame:\n" << comDynamics_.comPositionBaseFrame() << std::endl;
  std::cout << "comPositionBaseFrameDefault:\n" << comDynamics_.comPositionBaseFrame() << std::endl;

  std::cout << "comInertia:\n" << comDynamics_.comInertia() << std::endl;
  std::cout << "comInertiaDefault:\n" << comDynamics_.comInertia() << std::endl;
}

TEST_F(SwitchedModelTests, LF_Orientations) {
  joint_coordinate_t qJoints; qJoints.setZero();
  comkino_state_t x; x.setZero();


  std::cout << "\nqJoints:\n" << qJoints;
  Eigen::Matrix<double, switched_model::JOINT_COORDINATE_SIZE + 4, 1> extendedJointCoordinate;
  extendedJointCoordinate.template segment<3>(0) = qJoints.template segment<3>(0);
  extendedJointCoordinate(3) = 0.0;
  extendedJointCoordinate.template segment<3>(4) = qJoints.template segment<3>(3);
  extendedJointCoordinate(7) = 0.0;
  extendedJointCoordinate.template segment<3>(8) = qJoints.template segment<3>(6);
  extendedJointCoordinate(11) = 0.0;
  extendedJointCoordinate.template segment<3>(12) = qJoints.template segment<3>(9);
  extendedJointCoordinate(15) = 0.0;

  std::cout << "\nqJoints:\n" << qJoints;
  std::cout << "\nextendedJointPositions:\n" << x;
  const base_coordinate_t basePose = comDynamics_.calculateBasePose(switched_model::getComPose(x));
  matrix3_t o_R_b = switched_model::rotationMatrixBaseToOrigin(switched_model::getOrientation(basePose));
  std::cout << "\nBasePose:\n" << basePose;
  std::cout << "\nBase Orientation:\n" << o_R_b;

  {
    using namespace tpl;
    using trait_t = typename iit::rbd::tpl::TraitSelector<double>::Trait;
    // {std::cout << "\nType_fr_base_X_fr_LF_FOOT   \n" << typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_HAA tmp    ; std::cout << "\nType_fr_base_X_fr_LF_HAA    \n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_HFE tmp    ; std::cout << "\nType_fr_base_X_fr_LF_HFE    \n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_HIP tmp    ; std::cout << "\nType_fr_base_X_fr_LF_HIP    \n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_KFE tmp    ; std::cout << "\nType_fr_base_X_fr_LF_KFE    \n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_WHEEL tmp  ; std::cout << "\nType_fr_base_X_fr_LF_WHEEL  \n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    {typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_WHEEL_L tmp; std::cout << "\nType_fr_base_X_fr_LF_WHEEL_L\n" << tmp(extendedJointCoordinate).template topLeftCorner<3,3>();}
    std::cout << std::endl;
  }
}

TEST_F(SwitchedModelTests, EndeffectorOrientation) {
  joint_coordinate_t qJoints; qJoints.setZero();
  comkino_state_t x; x.setZero();


  std::cout << "\nqJoints:\n" << qJoints;
  Eigen::Matrix<double, switched_model::JOINT_COORDINATE_SIZE + 4, 1> extendedJointCoordinate;
  extendedJointCoordinate.template segment<3>(0) = qJoints.template segment<3>(0);
  extendedJointCoordinate(3) = 0.0;
  extendedJointCoordinate.template segment<3>(4) = qJoints.template segment<3>(3);
  extendedJointCoordinate(7) = 0.0;
  extendedJointCoordinate.template segment<3>(8) = qJoints.template segment<3>(6);
  extendedJointCoordinate(11) = 0.0;
  extendedJointCoordinate.template segment<3>(12) = qJoints.template segment<3>(9);
  extendedJointCoordinate(15) = 0.0;

  std::cout << "\nextendedJointPositions:\n" << extendedJointCoordinate;
  const base_coordinate_t basePose = comDynamics_.calculateBasePose(switched_model::getComPose(x));
  matrix3_t o_R_b = switched_model::rotationMatrixBaseToOrigin(switched_model::getOrientation(basePose));
  std::cout << "\nBasePose:\n" << basePose;
  std::cout << "\nBase Orientation:\n" << o_R_b;
  for (int footIdx = 0; footIdx < switched_model::NUM_CONTACT_POINTS; footIdx++) {
    const matrix3_t o_R_f = kinematics_.footOrientationInOriginFrame(footIdx, basePose, qJoints);
    std::cout << "\nFOOT-" << switched_model::feetNames[footIdx];
    std::cout << "\n -> FootOrientationRelativeToBaseFrame b_R_f:\n" << kinematics_.footOrientationRelativeToBaseFrame(footIdx, qJoints);
    std::cout << "\n -> Foot orientation o_R_f" << ":\n" << o_R_f;
    std::cout << std::endl;
    EXPECT_PRED2(matrixEquality_, o_R_f, o_R_b);
  }
}

TEST_F(SwitchedModelTests, EndeffectorAlignedYAxisRandomHFE_KFE) {
  /** Y Axis coincidence
   * 
   * With only the HFE, and KFE joints rotated, the y-axis of the foot should still be aligned with the
   * y-axis of the base, at any base pose.
   */
  auto posDist_ = std::uniform_real_distribution<scalar_t>(-20, 20);
  auto angleDist_ = std::uniform_real_distribution<scalar_t>(0.07, M_PI-0.1);
  auto randAngle = std::bind(angleDist_, std::ref(generator_));
  auto randPos = std::bind(posDist_, std::ref(generator_));

  comkino_state_t x; x.setZero();
  x[0] = randAngle();
  x[1] = randAngle();
  x[2] = randAngle();
  x[3] = randPos();
  x[4] = randPos();
  x[5] = randPos();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RF_HFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RF_KFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LF_HFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LF_KFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RH_HFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RH_KFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LH_HFE] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LH_KFE] = randAngle();
  joint_coordinate_t qJoints = x.segment<switched_model::JOINT_COORDINATE_SIZE>(switched_model::STATE_DIM - switched_model::JOINT_COORDINATE_SIZE);


  std::cout << "\nqJoints:\n" << qJoints;
  Eigen::Matrix<double, switched_model::JOINT_COORDINATE_SIZE + 4, 1> extendedJointCoordinate;
  extendedJointCoordinate.template segment<3>(0) = qJoints.template segment<3>(0);
  extendedJointCoordinate(3) = 0.0;
  extendedJointCoordinate.template segment<3>(4) = qJoints.template segment<3>(3);
  extendedJointCoordinate(7) = 0.0;
  extendedJointCoordinate.template segment<3>(8) = qJoints.template segment<3>(6);
  extendedJointCoordinate(11) = 0.0;
  extendedJointCoordinate.template segment<3>(12) = qJoints.template segment<3>(9);
  extendedJointCoordinate(15) = 0.0;
  std::cout << "\nextendedJointPositions:\n" << extendedJointCoordinate;

  const base_coordinate_t basePose = comDynamics_.calculateBasePose(switched_model::getComPose(x));
  matrix3_t o_R_b = switched_model::rotationMatrixBaseToOrigin(switched_model::getOrientation(basePose));
  vector3_t yAxis; yAxis << 0, 1, 0;
  std::cout << "\nBasePose:\n" << basePose;
  std::cout << "\nBase orientation\n:" << o_R_b;
  std::cout << "\nAxis:\n" << yAxis;
  for (int footIdx = 0; footIdx < switched_model::NUM_CONTACT_POINTS; footIdx++) {
    const matrix3_t o_R_f = kinematics_.footOrientationInOriginFrame(footIdx, basePose, qJoints);
    std::cout << "\n Foot-" << switched_model::feetNames[footIdx];
    std::cout << " orientation:\n" << o_R_f;
    std::cout << "\nFootOrientationRelativeToBaseFrame:\n" << kinematics_.footOrientationRelativeToBaseFrame(footIdx, qJoints);
    std::cout << std::endl;
    EXPECT_PRED2(matrixEquality_, o_R_f * yAxis, o_R_b * yAxis);
  }
}

TEST_F(SwitchedModelTests, EndeffectorAlignedXAxisRandomHAA) {
  /** X Axis coincidence
   * 
   * With only the HAA joints rotated, the x-axis of the foot should still be aligned with the
   * x-axis of the base, at any base pose.
   */
  auto posDist_ = std::uniform_real_distribution<scalar_t>(-20, 20);
  auto angleDist_ = std::uniform_real_distribution<scalar_t>(0.07, M_PI-0.1);
  auto randAngle = std::bind(angleDist_, std::ref(generator_));
  auto randPos = std::bind(posDist_, std::ref(generator_));

  comkino_state_t x; x.setZero();
  x[0] = randAngle();
  x[1] = randAngle();
  x[2] = randAngle();
  x[3] = randPos();
  x[4] = randPos();
  x[5] = randPos();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RF_HAA] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LF_HAA] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::RH_HAA] = randAngle();
  x[switched_model::BASE_COORDINATE_SIZE*2 + JointIdentifiers::LH_HAA] = randAngle();
  joint_coordinate_t qJoints = x.segment<switched_model::JOINT_COORDINATE_SIZE>(switched_model::STATE_DIM - switched_model::JOINT_COORDINATE_SIZE);

  std::cout << "\nqJoints:\n" << qJoints;
  Eigen::Matrix<double, switched_model::JOINT_COORDINATE_SIZE + 4, 1> extendedJointCoordinate;
  extendedJointCoordinate.template segment<3>(0) = qJoints.template segment<3>(0);
  extendedJointCoordinate(3) = 0.0;
  extendedJointCoordinate.template segment<3>(4) = qJoints.template segment<3>(3);
  extendedJointCoordinate(7) = 0.0;
  extendedJointCoordinate.template segment<3>(8) = qJoints.template segment<3>(6);
  extendedJointCoordinate(11) = 0.0;
  extendedJointCoordinate.template segment<3>(12) = qJoints.template segment<3>(9);
  extendedJointCoordinate(15) = 0.0;
  std::cout << "\nextendedJointPositions:\n" << extendedJointCoordinate;

  const base_coordinate_t basePose = comDynamics_.calculateBasePose(switched_model::getComPose(x));
  matrix3_t o_R_b = switched_model::rotationMatrixBaseToOrigin(switched_model::getOrientation(basePose));
  vector3_t xAxis; xAxis << 1, 0, 0;
  std::cout << "\nBasePose:\n" << basePose;
  std::cout << "\nBase orientation:\n" << o_R_b;
  std::cout << "\nAxis:\n" << xAxis << std::endl;
  for (int footIdx = 0; footIdx < switched_model::NUM_CONTACT_POINTS; footIdx++) {
    const matrix3_t o_R_f = kinematics_.footOrientationInOriginFrame(footIdx, basePose, qJoints);
    std::cout << "\nFoot orientation " << switched_model::feetNames[footIdx] << ":\n" << o_R_f;
    std::cout << "\nFootOrientationRelativeToBaseFrame:\n" << kinematics_.footOrientationRelativeToBaseFrame(footIdx, qJoints);
    std::cout << std::endl;
    EXPECT_PRED2(matrixEquality_,  o_R_f * xAxis, o_R_b * xAxis);
  }
}

/// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
