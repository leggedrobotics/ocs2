/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <iostream>

#include <gtest/gtest.h>

#include <ocs2_core/initialization/OperatingPoints.h>

class InitializationTest : public testing::Test {
 protected:
  static constexpr size_t stateDim_ = 3;
  static constexpr size_t inputDim_ = 2;

  using OperatingPoints = ocs2::OperatingPoints;
  using scalar_t = ocs2::scalar_t;
  using scalar_array_t = ocs2::scalar_array_t;
  using vector_t = ocs2::vector_t;
  using vector_array_t = ocs2::vector_array_t;

  InitializationTest() = default;

  vector_t input, nextState;
};

TEST_F(InitializationTest, SingleOperatingPoint) {
  const scalar_t t0 = 0.0;
  const scalar_t tf = 1.0;
  const vector_array_t xTraj = {vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.compute(t0, xTraj[0], tf, input, nextState);

  ASSERT_TRUE(nextState.isApprox(xTraj[0]));
  ASSERT_TRUE(input.isApprox(uTraj[0]));
}

TEST_F(InitializationTest, ZeroTimeInterval) {
  const scalar_t t0 = 0.0;
  const vector_array_t xTraj{vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.compute(t0, xTraj[0], t0, input, nextState);

  ASSERT_TRUE(nextState.isApprox(xTraj[0]));
  ASSERT_TRUE(input.isApprox(uTraj[0]));
}

TEST_F(InitializationTest, Trajectory) {
  constexpr size_t N = 20;
  scalar_array_t tTraj(N);
  scalar_t n = 0;
  std::generate(tTraj.begin(), tTraj.end(), [&n]() mutable { return n++; });
  vector_array_t xTraj(N);
  std::generate(xTraj.begin(), xTraj.end(), [&]() { return vector_t::Random(stateDim_); });
  vector_array_t uTraj(N);
  std::generate(uTraj.begin(), uTraj.end(), [&]() { return vector_t::Random(inputDim_); });
  OperatingPoints operatingPoints(tTraj, xTraj, uTraj);

  for (size_t i = 0; i < N - 1; i++) {
    operatingPoints.compute(tTraj[i], xTraj[i], tTraj[i + 1], input, nextState);
    ASSERT_TRUE(input.isApprox(uTraj[i]));
    ASSERT_TRUE(nextState.isApprox(xTraj[i + 1]));
  }
}
