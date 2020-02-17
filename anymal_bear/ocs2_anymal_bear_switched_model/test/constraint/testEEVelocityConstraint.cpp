//
// Created by rgrandia on 19.09.19.
//


#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityContraint.h>

#include "ocs2_anymal_bear_switched_model/core/AnymalBearCom.h"
#include "ocs2_anymal_bear_switched_model/core/AnymalBearKinematics.h"

TEST(TestEEVelocityConstraint, evaluate){
  using TestedConstraint = switched_model::EndEffectorVelocityConstraint;

  switched_model::EndEffectorVelocityConstraintSettings settings;
  settings.A.setIdentity(3, 3);
  settings.b.setZero(3);

  anymal::AnymalComAd anymalComAd;
  anymal::AnymalKinematicsAd anymalKinematicsAd;
  TestedConstraint eeVelocityConstraint(0, settings, anymalComAd, anymalKinematicsAd, true);

  // evaluation point
  double t = 0.0;
  TestedConstraint::input_vector_t u;
  TestedConstraint::state_vector_t x;
  u.setRandom();
  x.setRandom();

  auto approximation = eeVelocityConstraint.getLinearApproximation(t, x, u);
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
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
