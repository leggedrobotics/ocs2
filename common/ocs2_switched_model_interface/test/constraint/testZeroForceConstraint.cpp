//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

TEST(TestZeroForceConstraint, evaluate) {
  using TestedConstraint = switched_model::ZeroForceConstraint;
  TestedConstraint zeroForceConstraint(0);

  // evaluation point
  double t = 0.0;
  TestedConstraint::input_vector_t u;
  TestedConstraint::state_vector_t x;
  u.setZero();
  x.setZero();

  auto linearApproximation = zeroForceConstraint.getLinearApproximation(t, x, u);
  std::cout << "h" << std::endl;
  for (auto h : linearApproximation.constraintValues) {
    std::cout << h << std::endl;
  }

  std::cout << "dhdx" << std::endl;
  for (auto dhdx : linearApproximation.derivativeState) {
    std::cout << dhdx.transpose() << std::endl;
  }

  std::cout << "dhdu" << std::endl;
  for (auto dhdu : linearApproximation.derivativeInput) {
    std::cout << dhdu.transpose() << std::endl;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
