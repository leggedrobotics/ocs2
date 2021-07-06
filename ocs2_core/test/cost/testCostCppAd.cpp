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

#include <ocs2_core/cost/StateCostCppAd.h>
#include <ocs2_core/cost/StateInputCostCppAd.h>
#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>

class TestStateCost : public ocs2::StateCostCppAd {
 public:
  TestStateCost() { initialize(2, 0, "TestStateCost", "/tmp/ocs2", true, false); }
  ~TestStateCost() override = default;
  TestStateCost* clone() const override { return new TestStateCost(*this); }

  ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state,
                                 const ocs2::ad_vector_t& parameters) const override {
    ocs2::ad_scalar_t cost;
    const ocs2::matrix_t Q = (ocs2::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    cost = ocs2::ad_scalar_t(0.5) * state.dot(Q.cast<ocs2::ad_scalar_t>() * state);
    return cost;
  }

 private:
  TestStateCost(const TestStateCost& other) = default;
};

TEST(TestStateCostCppAd, getValue) {
  TestStateCost cost;
  const ocs2::TargetTrajectories desiredTrajectory;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Ones(2);

  const auto val = cost.getValue(t, x, desiredTrajectory, ocs2::PreComputation());
  const auto approx = cost.getQuadraticApproximation(t, x, desiredTrajectory, ocs2::PreComputation());

  EXPECT_NEAR(val, 1.5, 1e-6);
  EXPECT_NEAR(approx.f, val, 1e-6);
  EXPECT_TRUE(approx.dfdx.isApprox((ocs2::matrix_t(2, 1) << 1, 2).finished()));
  EXPECT_TRUE(approx.dfdxx.isApprox((ocs2::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
}

class TestStateInputCost : public ocs2::StateInputCostCppAd {
 public:
  TestStateInputCost() { initialize(2, 1, 0, "TestStateInputCost", "/tmp/ocs2", true, false); }
  ~TestStateInputCost() override = default;
  TestStateInputCost* clone() const override { return new TestStateInputCost(*this); }

  ocs2::ad_scalar_t costFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                 const ocs2::ad_vector_t& parameters) const override {
    ocs2::ad_scalar_t cost;
    const ocs2::matrix_t Q = (ocs2::matrix_t(2, 2) << 1.0, 0.0, 0.0, 2.0).finished();
    const ocs2::matrix_t R = (ocs2::matrix_t(1, 1) << 1.0).finished();
    const ocs2::matrix_t P = (ocs2::matrix_t(1, 2) << 1.0, 1.0).finished();
    cost = ocs2::ad_scalar_t(0.5) * state.dot(Q.cast<ocs2::ad_scalar_t>() * state);
    cost += ocs2::ad_scalar_t(0.5) * input.dot(R.cast<ocs2::ad_scalar_t>() * input);
    cost += input.dot(P.cast<ocs2::ad_scalar_t>() * state);
    return cost;
  }

 private:
  TestStateInputCost(const TestStateInputCost& other) = default;
};

TEST(TestStateInputCostCppAd, getValue) {
  TestStateInputCost cost;
  const ocs2::TargetTrajectories desiredTrajectory;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Ones(2);
  const ocs2::vector_t u = ocs2::vector_t::Ones(1);

  const auto val = cost.getValue(t, x, u, desiredTrajectory, ocs2::PreComputation());
  const auto approx = cost.getQuadraticApproximation(t, x, u, desiredTrajectory, ocs2::PreComputation());

  EXPECT_NEAR(val, 4.0, 1e-6);
  EXPECT_NEAR(approx.f, val, 1e-6);
  EXPECT_TRUE(approx.dfdx.isApprox((ocs2::matrix_t(2, 1) << 2, 3).finished()));
  EXPECT_TRUE(approx.dfdu.isApprox((ocs2::matrix_t(1, 1) << 3).finished()));
  EXPECT_TRUE(approx.dfdxx.isApprox((ocs2::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
  EXPECT_TRUE(approx.dfduu.isApprox((ocs2::matrix_t(1, 1) << 1).finished()));
  EXPECT_TRUE(approx.dfdux.isApprox((ocs2::matrix_t(1, 2) << 1, 1).finished()));
}

class TestGNStateInputCost : public ocs2::StateInputCostGaussNewtonAd {
 public:
  TestGNStateInputCost() { initialize(2, 1, 0, "TestGNStateInputCost", "/tmp/ocs2", true, false); }
  ~TestGNStateInputCost() override = default;
  TestGNStateInputCost* clone() const override { return new TestGNStateInputCost(*this); }

  template <typename SCALAR_T>
  static Eigen::Matrix<SCALAR_T, -1, 1> costVector(SCALAR_T time, const Eigen::Matrix<SCALAR_T, -1, 1>& state,
                                                   const Eigen::Matrix<SCALAR_T, -1, 1>& input) {
    Eigen::Matrix<SCALAR_T, -1, 1> costVector(4);
    costVector << state(0), state(1), time * input(0), state(0) + input(0);
    return costVector;
  }

  ocs2::ad_vector_t costVectorFunction(ocs2::ad_scalar_t time, const ocs2::ad_vector_t& state, const ocs2::ad_vector_t& input,
                                       const ocs2::ad_vector_t& parameters) const override {
    return costVector(time, state, input);
  }

 private:
  TestGNStateInputCost(const TestGNStateInputCost& other) = default;
};

TEST(TestGNStateInputCostCppAd, getValue) {
  TestGNStateInputCost cost;
  const ocs2::TargetTrajectories desiredTrajectory;

  const ocs2::scalar_t t = 0.4;
  const ocs2::vector_t x = (ocs2::vector_t(2) << 0.1, 0.2).finished();
  const ocs2::vector_t u = (ocs2::vector_t(1) << 0.3).finished();

  ocs2::vector_t f = TestGNStateInputCost::costVector(t, x, u);

  const auto val = cost.getValue(t, x, u, desiredTrajectory, ocs2::PreComputation());
  const auto approx = cost.getQuadraticApproximation(t, x, u, desiredTrajectory, ocs2::PreComputation());

  // cost = 0.5 * |f|^2 = 0.5*x(0)^2 + 0.5*x(1)^2 + 0.5*(t*u(0))^2 + 0.5*(x(0) + u(0))^2
  ASSERT_DOUBLE_EQ(val, 0.5 * f.squaredNorm());
  ASSERT_DOUBLE_EQ(approx.f, val);
  ASSERT_DOUBLE_EQ(approx.dfdx(0), 2.0 * x(0) + u(0));
  ASSERT_DOUBLE_EQ(approx.dfdx(1), x(1));
  ASSERT_DOUBLE_EQ(approx.dfdu(0), (t * t + 1.0) * u(0) + x(0));
  ASSERT_DOUBLE_EQ(approx.dfdxx(0, 0), 2.0);
  ASSERT_DOUBLE_EQ(approx.dfdxx(0, 1), 0.0);
  ASSERT_DOUBLE_EQ(approx.dfdxx(1, 0), approx.dfdxx(0, 1));
  ASSERT_DOUBLE_EQ(approx.dfdxx(1, 1), 1.0);
  ASSERT_DOUBLE_EQ(approx.dfdux(0, 0), 1.0);
  ASSERT_DOUBLE_EQ(approx.dfdux(0, 1), 0.0);
  ASSERT_DOUBLE_EQ(approx.dfduu(0, 0), (t * t + 1.0));
}
