//
// Created by rgrandia on 18.02.21.
//

#include <gtest/gtest.h>

#include "ocs2_sqp/QrConstraintProjection.h"

#include <ocs2_qp_solver/test/testProblemsGeneration.h>

TEST(test_projection, testProjection) {
  const auto constraint = ocs2::qp_solver::getRandomConstraints(4, 3, 2);
  auto projection = ocs2::qrConstraintProjection(constraint);

  // range of Pu is in null-space of D
  ASSERT_TRUE((constraint.dfdu * projection.dfdu).isZero());

  // D * Px cancels the C term
  ASSERT_TRUE((constraint.dfdx + constraint.dfdu * projection.dfdx).isZero());

  // D * Pe cancels the e term
  ASSERT_TRUE((constraint.f + constraint.dfdu * projection.f).isZero());
}