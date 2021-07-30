//
// Created by rgrandia on 26.07.21.
//

#include <gtest/gtest.h>

#include "ocs2_switched_model_interface/cost/JointLimitsSoftConstraint.h"

using namespace switched_model;

TEST(TestJointLimits, quadraticApproximation) {
  joint_coordinate_t upper = joint_coordinate_t::Constant(0.5);
  joint_coordinate_t lower = joint_coordinate_t::Constant(-0.5);
  const scalar_t mu = 0.1;
  const scalar_t delta = 0.05;

  JointLimitsSoftConstraint jointLimitsSoftConstraint({lower, upper}, {mu, delta});

  joint_coordinate_t middle = 0.5 * (lower + upper);
  auto value = jointLimitsSoftConstraint.getValue(middle);
  auto quadApprox = jointLimitsSoftConstraint.getQuadraticApproximation(middle);

  // Gradient is zero in middle of limits
  ASSERT_DOUBLE_EQ(quadApprox.dfdx.norm(), 0.0);

  // Value is consistent
  ASSERT_DOUBLE_EQ(quadApprox.f, value);
}

TEST(TestJointLimits, infiniteLimits) {
  joint_coordinate_t upper = joint_coordinate_t::Constant(1e30);
  joint_coordinate_t lower = joint_coordinate_t::Constant(-1e30);
  const scalar_t mu = 0.1;
  const scalar_t delta = 0.05;

  JointLimitsSoftConstraint jointLimitsSoftConstraint({lower, upper}, {mu, delta});

  joint_coordinate_t middle = 0.5 * (lower + upper);
  auto value = jointLimitsSoftConstraint.getValue(middle);
  auto quadApprox = jointLimitsSoftConstraint.getQuadraticApproximation(middle);

  // Gradient and curvature are both almost zero
  ASSERT_LT(quadApprox.dfdx.norm(), 1e-30);
  ASSERT_LT(quadApprox.dfdxx.norm(), 1e-30);
}