//
// Created by rgrandia on 19.09.19.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

TEST(TestZeroForceConstraint, evaluate) {
  using TestedConstraint = switched_model::ZeroForceConstraint;
  TestedConstraint zeroForceConstraint;
  zeroForceConstraint.setContactFlags({false, true, false, true});

  // evaluation point
  double t = 0.0;
  switched_model::input_vector_t u;
  switched_model::state_vector_t x;
  u.setZero();
  x.setZero();

  auto linearApproximation = zeroForceConstraint.getLinearApproximation(t, x, u);
  std::cout << "h: " << linearApproximation.f.transpose() << std::endl;
  std::cout << "dhdx: \n" << linearApproximation.dfdx << std::endl;
  std::cout << "dhdu: \n" << linearApproximation.dfdu << std::endl;
}
