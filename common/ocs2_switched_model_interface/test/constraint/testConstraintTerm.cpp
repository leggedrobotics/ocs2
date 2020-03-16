//
// Created by rgrandia on 28.03.19.
//

#include <gtest/gtest.h>

#include "testConstraintTerm.h"

TEST(TestConstraintTerm, Initialization) {
  TestEmptyConstraint emptyConstraint;
  TestLinearConstraint linearConstraint;

  ASSERT_EQ(emptyConstraint.getNumConstraints(0.0), 0);
  ASSERT_EQ(linearConstraint.getNumConstraints(0.0), 2);
}

TEST(TestConstraintTerm, getValue) {
  TestLinearConstraint linearConstraint;

  // evaluation point
  double t = 0.0;
  TestLinearConstraint::input_vector_t u;
  TestLinearConstraint::state_vector_t x;
  u.setZero();
  x.setZero();

  auto linearValues = linearConstraint.getValue(t, x, u);
  ASSERT_EQ(linearValues[0], 1);
  ASSERT_EQ(linearValues[1], 2);
}

TEST(TestConstraintTerm, getDerivatives) {
  TestLinearConstraint linearConstraint;

  // evaluation point
  double t = 0.0;
  TestLinearConstraint::input_vector_t u;
  TestLinearConstraint::state_vector_t x;
  u.setZero();
  x.setZero();

  auto linearApproximation = linearConstraint.getLinearApproximation(t, x, u);
  ASSERT_EQ(linearApproximation.constraintValues[0], 1);
  ASSERT_EQ(linearApproximation.constraintValues[1], 2);
  ASSERT_EQ(linearApproximation.derivativeState[0].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeState[1].sum(), 3);
  ASSERT_EQ(linearApproximation.derivativeInput[0].sum(), 0);
  ASSERT_EQ(linearApproximation.derivativeInput[1].sum(), 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
