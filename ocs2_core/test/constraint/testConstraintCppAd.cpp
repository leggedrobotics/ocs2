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

#include <ocs2_core/constraint/StateConstraintCppAd.h>
#include <ocs2_core/constraint/StateInputConstraintCppAd.h>

class TestStateConstraint : public ocs2::StateConstraintCppAd {
 public:
  TestStateConstraint() : ocs2::StateConstraintCppAd(ocs2::ConstraintOrder::Quadratic) {
    initialize(2, 0, "TestStateConstraint", "/tmp/ocs2", true, false);
  }
  ~TestStateConstraint() override = default;
  TestStateConstraint* clone() const override { return new TestStateConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 2; }

  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                       const ocs2::ad_vector_t& parameters) const override {
    ocs2::ad_vector_t constraint(2);
    ocs2::matrix_t Q = (ocs2::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    constraint(0) = ocs2::ad_scalar_t(0.5) * state.dot(Q.cast<ocs2::ad_scalar_t>() * state);
    constraint(1) = 3.0 * state(0) + 4.0 * state(1);
    return constraint;
  }

 private:
  TestStateConstraint(const TestStateConstraint& other) = default;
};

TEST(TestStateConstraintCppAd, getValue) {
  TestStateConstraint constraint;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Ones(2);

  const auto val = constraint.getValue(t, x, ocs2::PreComputation());
  const auto lin = constraint.getLinearApproximation(t, x, ocs2::PreComputation());
  const auto quad = constraint.getQuadraticApproximation(t, x, ocs2::PreComputation());

  EXPECT_TRUE(val.isApprox((ocs2::vector_t(2) << 1.5, 7).finished()));
  EXPECT_TRUE(lin.f.isApprox(val));
  EXPECT_TRUE(lin.dfdx.isApprox((ocs2::matrix_t(2, 2) << 1, 2, 3, 4).finished()));
  EXPECT_TRUE(quad.f.isApprox(lin.f));
  EXPECT_TRUE(quad.dfdx.isApprox(lin.dfdx));
  EXPECT_TRUE(quad.dfdxx[0].isApprox((ocs2::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(quad.dfdxx[1].isZero());
}

class TestStateInputConstraint : public ocs2::StateInputConstraintCppAd {
 public:
  TestStateInputConstraint() : ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Quadratic) {
    initialize(2, 1, 0, "TestStateInputConstraint", "/tmp/ocs2", true, false);
  }
  ~TestStateInputConstraint() override = default;
  TestStateInputConstraint* clone() const override { return new TestStateInputConstraint(*this); }

  size_t getNumConstraints(ocs2::scalar_t time) const override { return 2; }

  ocs2::ad_vector_t constraintFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override {
    ocs2::ad_vector_t constraint(2);
    ocs2::matrix_t Q = (ocs2::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    constraint(0) = ocs2::ad_scalar_t(0.5) * state.dot(Q.cast<ocs2::ad_scalar_t>() * state) + input(0);
    constraint(1) = 3.0 * state(0) + 4.0 * state(1) - input(0) * input(0);
    return constraint;
  }

 private:
  TestStateInputConstraint(const TestStateInputConstraint& other) = default;
};

TEST(TestStateInputConstraintCppAd, getValue) {
  TestStateInputConstraint constraint;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Ones(2);
  const ocs2::vector_t u = ocs2::vector_t::Ones(1);

  const auto val = constraint.getValue(t, x, u, ocs2::PreComputation());
  const auto lin = constraint.getLinearApproximation(t, x, u, ocs2::PreComputation());
  const auto quad = constraint.getQuadraticApproximation(t, x, u, ocs2::PreComputation());

  EXPECT_TRUE(val.isApprox((ocs2::vector_t(2) << 2.5, 6).finished()));
  EXPECT_TRUE(lin.f.isApprox(val));
  EXPECT_TRUE(lin.dfdx.isApprox((ocs2::matrix_t(2, 2) << 1, 2, 3, 4).finished()));
  EXPECT_TRUE(lin.dfdu.isApprox((ocs2::matrix_t(2, 1) << 1, -2).finished()));
  EXPECT_TRUE(quad.f.isApprox(lin.f));
  EXPECT_TRUE(quad.dfdx.isApprox(lin.dfdx));
  EXPECT_TRUE(quad.dfdu.isApprox(lin.dfdu));
  EXPECT_TRUE(quad.dfdxx[0].isApprox((ocs2::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(quad.dfdxx[1].isZero());
  EXPECT_TRUE(quad.dfdux[0].isZero());
  EXPECT_TRUE(quad.dfdux[1].isZero());
  EXPECT_TRUE(quad.dfduu[0].isZero());
  EXPECT_TRUE(quad.dfduu[1].isApprox((ocs2::matrix_t(1, 1) << -2).finished()));
}
