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
  const ocs2::CostDesiredTrajectories desiredTrajectory;

  const ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = ocs2::vector_t::Ones(2);

  const auto val = cost.getValue(t, x, desiredTrajectory);
  const auto approx = cost.getQuadraticApproximation(t, x, desiredTrajectory);

  EXPECT_NEAR(val, 1.5, 1e-6);
  EXPECT_NEAR(approx.f, val, 1e-6);
  EXPECT_TRUE(approx.dfdx.isApprox((ocs2::matrix_t(2, 1) << 1, 2).finished()));
  EXPECT_TRUE(approx.dfdxx.isApprox((ocs2::matrix_t(2, 2) << 1, 0, 0, 2).finished()));
}
