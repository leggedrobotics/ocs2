//
// Created by rgrandia on 12.05.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_models/DynamicsHelpers.h>

#include <ocs2_switched_model_interface/core/Rotations.h>

using namespace switched_model;

TEST(TestInertiaInverse, inv33sym) {
  using anymal::inv33sym;

  matrix3_t Ahalf = matrix3_t::Random();
  matrix3_t A = Ahalf.transpose() * Ahalf;
  matrix3_t AinvTest = inv33sym(A);
  matrix3_t AinvCheck = A.inverse();

  ASSERT_TRUE(AinvTest.isApprox(AinvCheck));
}

TEST(TestInertiaInverse, solve33sym) {
  using anymal::solve33sym;

  matrix3_t Ahalf = matrix3_t::Random();
  vector3_t b = vector3_t::Random();
  matrix3_t A = Ahalf.transpose() * Ahalf;

  vector3_t solveTest = solve33sym(A, b);
  vector3_t solveCheck = A.lu().solve(b);

  ASSERT_TRUE(solveTest.isApprox(solveCheck));
}

TEST(TestInertiaInverse, inertiaTensorSolve) {
  using anymal::inertiaTensorSolve;

  // Construct a inertia tensor
  double m = 10.0;
  matrix3_t Ihalf = matrix3_t::Random();
  vector3_t comVector = vector3_t::Random();
  matrix3_t crossTerm = crossProductMatrix<scalar_t>(m * comVector);
  matrix6_t M;
  M << m * matrix3_t::Identity(), crossTerm.transpose(), crossTerm, Ihalf.transpose() * Ihalf + 1.0 / m * crossTerm * crossTerm.transpose();

  // Check if the eigenvalues are valid
  ASSERT_GT(M.eigenvalues().real().minCoeff(), 0.0);

  vector6_t b = vector6_t::Random();

  vector6_t solveTest = inertiaTensorSolve(M, b);
  vector6_t solveCheck = M.lu().solve(b);

  ASSERT_TRUE(solveTest.isApprox(solveCheck));
}