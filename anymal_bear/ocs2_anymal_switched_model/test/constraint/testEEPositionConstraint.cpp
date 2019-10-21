//
// Created by rgrandia on 19.09.19.
//


#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h>

#include "ocs2_anymal_switched_model/core/AnymalKinematics.h"
#include "ocs2_anymal_switched_model/core/AnymalCom.h"

TEST(TestEEPositionConstraint, evaluate){
  using TestedConstraint = switched_model::EndEffectorPositionConstraint<24, 24>;

  switched_model::EndEffectorPositionConstraintSettings settings;
  settings.A.setIdentity(3, 3);
  settings.b.setZero(3);

  anymal::AnymalComAd anymalComAd;
  anymal::AnymalKinematicsAd anymalKinematicsAd;
  TestedConstraint eePositionConstraint(0, settings, anymalComAd, anymalKinematicsAd, true);

  // evaluation point
  double t = 0.0;
  TestedConstraint::input_vector_t u;
  TestedConstraint::state_vector_t x;
  u.setRandom();
  x.setRandom();

  auto approximation = eePositionConstraint.getQuadraticApproximation(t, x, u);
  std::cout << "h" << std::endl;
  for (auto h : approximation.constraintValues){
    std::cout << h << std::endl;
  }

  std::cout << "dhdx" << std::endl;
  for (auto dhdx : approximation.derivativeState){
    std::cout << dhdx.transpose() << std::endl;
  }

  std::cout << "dhdu" << std::endl;
  for (auto dhdu : approximation.derivativeInput){
    std::cout << dhdu.transpose() << std::endl;
  }

  std::cout << "ddhdxdx" << std::endl;
  int count = 0;
  for (auto ddhdxdx : approximation.secondDerivativesState){
    std::cout << "\t ddhdxdx[" << count << "]" << std::endl;
    std::cout << ddhdxdx << std::endl;
  }

  std::cout << "ddhdudu" << std::endl;
  count = 0;
  for (auto ddhdudu : approximation.secondDerivativesInput){
    std::cout << "\t ddhdudu[" << count << "]" << std::endl;
    std::cout << ddhdudu << std::endl;
  }

  std::cout << "ddhdudx" << std::endl;
  count = 0;
  for (auto ddhdudx : approximation.derivativesInputState){
    std::cout << "\t ddhdudx[" << count << "]" << std::endl;
    std::cout << ddhdudx << std::endl;
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
