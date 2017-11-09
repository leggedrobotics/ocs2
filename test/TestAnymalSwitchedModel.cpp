/*!
* @file   TestAnymalSwitchedModel.cpp
* @author Jan Carius
* @date   Nov, 2017
*/

#include <gtest/gtest.h>

#include "TestAnymalSwitchedModel.hpp"

namespace anymal {
class SwitchedModelTests : public ::testing::Test,
                           public TestAnymalSwitchedModel {
public:
  typedef AnymalKinematics::generalized_coordinate_t generalized_coordinate_t;
  typedef AnymalKinematics::joint_coordinate_t joint_coordinate_t;
};


TEST_F(SwitchedModelTests, Kinematics){

  generalized_coordinate_t gen_cor;
  gen_cor.setZero();
  std::cout << "Generalized coordinate:\n" << gen_cor.transpose() << std::endl;

  kinematics_.update(gen_cor);

  Eigen::Vector3d footPosition_LF;
  kinematics_.footPositionBaseFrame(0, footPosition_LF);
  std::cout << "Foot position LF:\n" << footPosition_LF << std::endl;

  Eigen::Vector3d footPosition_RF;
  kinematics_.footPositionBaseFrame(1, footPosition_RF);
  std::cout << "Foot position RF:\n" << footPosition_RF << std::endl;

  Eigen::Vector3d footPosition_LH;
  kinematics_.footPositionBaseFrame(2, footPosition_LH);
  std::cout << "Foot position LH:\n" << footPosition_LH << std::endl;

  Eigen::Vector3d footPosition_RH;
  kinematics_.footPositionBaseFrame(3, footPosition_RH);
  std::cout << "Foot position RH:\n" << footPosition_RH << std::endl;


  Eigen::Matrix<double,6,12> footJacobian_LF;
  kinematics_.footJacobainBaseFrame(0, footJacobian_LF);
  std::cout << "Foot jacobian LF:\n" << footJacobian_LF << std::endl;

  Eigen::Matrix<double,6,12> footJacobian_RF;
  kinematics_.footJacobainBaseFrame(1, footJacobian_RF);
  std::cout << "Foot jacobian RF:\n" << footJacobian_RF << std::endl;

  Eigen::Matrix<double,6,12> footJacobian_LH;
  kinematics_.footJacobainBaseFrame(2, footJacobian_LH);
  std::cout << "Foot jacobian LH:\n" << footJacobian_LH << std::endl;

  Eigen::Matrix<double,6,12> footJacobian_RH;
  kinematics_.footJacobainBaseFrame(3, footJacobian_RH);
  std::cout << "Foot jacobian RH:\n" << footJacobian_RH << std::endl;

}

TEST_F(SwitchedModelTests, Dynamics){
  joint_coordinate_t joint_cor;
  joint_cor.setZero();
  std::cout << "Joint coordinates:\n" << joint_cor.transpose() << std::endl;

  std::cout << "comPositionBaseFrame:\n" << comDynamics_.comPositionBaseFrame(joint_cor) << std::endl;

  std::cout << "comInertia:\n" << comDynamics_.comInertia(joint_cor) << std::endl;


}

} // namespace anymal
