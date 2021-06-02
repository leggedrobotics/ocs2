//
// Created by rgrandia on 12.05.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_models/RobcogenHelpers.h>

TEST(TestInertiaInverse, inv33sym) {
  using anymal::robcogen_helpers::inv33sym;

  switched_model::matrix3_t Ahalf = switched_model::matrix3_t::Random();
  switched_model::matrix3_t A = Ahalf.transpose() * Ahalf;
  switched_model::matrix3_t AinvTest = inv33sym(A);
  switched_model::matrix3_t AinvCheck = A.inverse();

  ASSERT_TRUE(AinvTest.isApprox(AinvCheck));
}

TEST(TestInertiaInverse, solve33sym) {
  using anymal::robcogen_helpers::solve33sym;

  switched_model::matrix3_t Ahalf = switched_model::matrix3_t::Random();
  switched_model::vector3_t b = switched_model::vector3_t::Random();
  switched_model::matrix3_t A = Ahalf.transpose() * Ahalf;

  switched_model::vector3_t solveTest = solve33sym(A, b);
  switched_model::vector3_t solveCheck = A.lu().solve(b);

  ASSERT_TRUE(solveTest.isApprox(solveCheck));
}

TEST(TestInertiaInverse, inertiaTensorSolve) {
  using anymal::robcogen_helpers::inertiaTensorSolve;

  // Construct a inertia tensor
  double m = 10.0;
  switched_model::matrix3_t Ihalf = switched_model::matrix3_t::Random();
  switched_model::vector3_t comVector = switched_model::vector3_t::Random();
  switched_model::matrix3_t crossTerm = switched_model::crossProductMatrix<switched_model::scalar_t>(m * comVector);
  switched_model::matrix6_t M;
  M << Ihalf.transpose() * Ihalf + 1.0 / m * crossTerm * crossTerm.transpose(), crossTerm, crossTerm.transpose(),
      m * switched_model::matrix3_t::Identity();

  // Check if the eigenvalues are valid
  ASSERT_GT(M.eigenvalues().real().minCoeff(), 0.0);

  switched_model::vector6_t b = switched_model::vector6_t::Random();

  switched_model::vector6_t solveTest = inertiaTensorSolve(M, b);
  switched_model::vector6_t solveCheck = M.lu().solve(b);

  ASSERT_TRUE(solveTest.isApprox(solveCheck));
}