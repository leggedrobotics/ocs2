/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>

TEST(TestLinearConstraint, testLinearStateInputConstraint) {
  const ocs2::vector_t e = ocs2::vector_t::Random(3);
  const ocs2::matrix_t C = ocs2::matrix_t::Random(3, 2);
  const ocs2::matrix_t D = ocs2::matrix_t::Random(3, 1);
  ocs2::LinearStateInputConstraint constraint(e, C, D);

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Random(2);
  const ocs2::vector_t u = ocs2::vector_t::Random(1);

  const auto value = constraint.getValue(t, x, u, ocs2::PreComputation());
  const auto approx = constraint.getLinearApproximation(t, x, u, ocs2::PreComputation());
  EXPECT_EQ(constraint.getNumConstraints(t), value.rows());
  EXPECT_TRUE(value.isApprox(C * x + D * u + e));
  EXPECT_TRUE(approx.f.isApprox(value));
  EXPECT_TRUE(approx.dfdx.isApprox(C));
  EXPECT_TRUE(approx.dfdu.isApprox(D));
}

TEST(TestLinearConstraint, testLinearStateConstraint) {
  const ocs2::vector_t e = ocs2::vector_t::Random(3);
  const ocs2::matrix_t C = ocs2::matrix_t::Random(3, 2);
  ocs2::LinearStateConstraint constraint(e, C);

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Random(2);

  const auto value = constraint.getValue(t, x, ocs2::PreComputation());
  const auto approx = constraint.getLinearApproximation(t, x, ocs2::PreComputation());
  EXPECT_EQ(constraint.getNumConstraints(t), value.rows());
  EXPECT_TRUE(value.isApprox(C * x + e));
  EXPECT_TRUE(approx.f.isApprox(value));
  EXPECT_TRUE(approx.dfdx.isApprox(C));
}
