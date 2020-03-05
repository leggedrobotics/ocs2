/*!
 * @file   TestAnymalWheelsSwitchedModel.cpp
 * @author Marko Bjelonic
 * @date   Nov 27, 2019
 */

#include <gtest/gtest.h>

#include "include/TestAnymalWheelsSwitchedModel.h"

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

/// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
