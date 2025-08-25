/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include "ocs2_oc/approximate_model/ChangeOfInputVariables.h"
#include "ocs2_oc/test/testProblemsGeneration.h"

using namespace ocs2;

namespace {
scalar_t evaluate(const ScalarFunctionQuadraticApproximation& scalarFunctionQuadraticApproximation, const vector_t& dx,
                  const vector_t& du) {
  // Short hand
  const auto& Q = scalarFunctionQuadraticApproximation.dfdxx;
  const auto& R = scalarFunctionQuadraticApproximation.dfduu;
  const auto& P = scalarFunctionQuadraticApproximation.dfdux;
  const auto& c = scalarFunctionQuadraticApproximation.f;

  vector_t q = scalarFunctionQuadraticApproximation.dfdx;
  q.noalias() += 0.5 * Q * dx;
  q.noalias() += P.transpose() * du;

  vector_t r = scalarFunctionQuadraticApproximation.dfdu;
  r.noalias() += 0.5 * R * du;

  return c + q.dot(dx) + r.dot(du);
}

vector_t evaluate(const VectorFunctionLinearApproximation& vectorFunctionLinearApproximation, const vector_t& dx, const vector_t& du) {
  // Short hand
  const auto& A = vectorFunctionLinearApproximation.dfdx;
  const auto& B = vectorFunctionLinearApproximation.dfdu;

  vector_t b = vectorFunctionLinearApproximation.f;
  b.noalias() += A * dx;
  b.noalias() += B * du;
  return b;
}
}  // namespace

TEST(quadratic_change_of_input_variables, noPx_noU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const auto quadratic = getRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  changeOfInputVariables(quadraticProjected, Pu);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = evaluate(quadratic, dx, Pu * du_tilde);
  const scalar_t projected = evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(quadratic_change_of_input_variables, withPx) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const auto quadratic = getRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  changeOfInputVariables(quadraticProjected, Pu, Px);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = evaluate(quadratic, dx, Pu * du_tilde + Px * dx);
  const scalar_t projected = evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_NEAR(unprojected, projected, 1e-10);
}

TEST(quadratic_change_of_input_variables, withU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const vector_t u0 = vector_t::Random(m);
  const auto quadratic = getRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  changeOfInputVariables(quadraticProjected, Pu, matrix_t(), u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = evaluate(quadratic, dx, Pu * du_tilde + u0);
  const scalar_t projected = evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(quadratic_change_of_input_variables, bothPx_U0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const vector_t u0 = vector_t::Random(m);
  const auto quadratic = getRandomCost(n, m);

  // Apply change of variables
  auto quadraticProjected = quadratic;
  changeOfInputVariables(quadraticProjected, Pu, Px, u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const scalar_t unprojected = evaluate(quadratic, dx, Pu * du_tilde + Px * dx + u0);
  const scalar_t projected = evaluate(quadraticProjected, dx, du_tilde);
  ASSERT_DOUBLE_EQ(unprojected, projected);
}

TEST(linear_change_of_input_variables, noPx_noU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const auto linear = getRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  changeOfInputVariables(linearProjected, Pu);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = evaluate(linear, dx, Pu * du_tilde);
  const vector_t projected = evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, withPx) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const auto linear = getRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  changeOfInputVariables(linearProjected, Pu, Px);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = evaluate(linear, dx, Pu * du_tilde + Px * dx);
  const vector_t projected = evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, withU0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const vector_t u0 = vector_t::Random(m);
  const auto linear = getRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  changeOfInputVariables(linearProjected, Pu, matrix_t(), u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = evaluate(linear, dx, Pu * du_tilde + u0);
  const vector_t projected = evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}

TEST(linear_change_of_input_variables, bothPx_U0) {
  const int n = 4;
  const int m = 3;
  const int p = 2;

  // Create change of variables
  const matrix_t Pu = matrix_t::Random(m, p);
  const matrix_t Px = matrix_t::Random(m, n);
  const vector_t u0 = vector_t::Random(m);
  const auto linear = getRandomDynamics(n, m);

  // Apply change of variables
  auto linearProjected = linear;
  changeOfInputVariables(linearProjected, Pu, Px, u0);

  // Evaluation point
  const vector_t du_tilde = vector_t::Random(p);
  const vector_t dx = vector_t::Random(n);

  // Evaluate and compare
  const vector_t unprojected = evaluate(linear, dx, Pu * du_tilde + Px * dx + u0);
  const vector_t projected = evaluate(linearProjected, dx, du_tilde);
  ASSERT_TRUE(unprojected.isApprox(projected));
}