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

  scalar_array_t timeTrajectory;
  vector_array_t stateTrajectory;
  vector_array_t inputTrajectory;
};

TEST_F(InitializationTest, SingleOperatingPoint) {
  const scalar_t t0 = 0.0;
  const scalar_t tf = 1.0;
  const vector_array_t xTraj = {vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.getSystemOperatingTrajectories(xTraj[0], t0, tf, timeTrajectory, stateTrajectory, inputTrajectory);

  ASSERT_EQ(timeTrajectory.size(), 2);
  ASSERT_EQ(stateTrajectory.size(), 2);
  ASSERT_EQ(inputTrajectory.size(), 2);

  ASSERT_DOUBLE_EQ(timeTrajectory.front(), t0);
  ASSERT_TRUE(stateTrajectory.front().isApprox(xTraj[0]));
  ASSERT_TRUE(inputTrajectory.front().isApprox(uTraj[0]));

  ASSERT_DOUBLE_EQ(timeTrajectory.back(), tf);
  ASSERT_TRUE(stateTrajectory.back().isApprox(xTraj[0]));
  ASSERT_TRUE(inputTrajectory.back().isApprox(uTraj[0]));
}

TEST_F(InitializationTest, ZeroTimeInterval) {
  const scalar_t t0 = 0.0;
  const vector_array_t xTraj{vector_t::Random(stateDim_)};
  const vector_array_t uTraj = {vector_t::Random(inputDim_)};
  OperatingPoints operatingPoints({t0}, xTraj, uTraj);
  operatingPoints.getSystemOperatingTrajectories(xTraj[0], t0, t0, timeTrajectory, stateTrajectory, inputTrajectory);

  ASSERT_EQ(timeTrajectory.size(), 1);
  ASSERT_EQ(stateTrajectory.size(), 1);
  ASSERT_EQ(inputTrajectory.size(), 1);

  ASSERT_DOUBLE_EQ(timeTrajectory.front(), t0);
  ASSERT_TRUE(stateTrajectory.front().isApprox(xTraj[0]));
  ASSERT_TRUE(inputTrajectory.front().isApprox(uTraj[0]));
}

TEST_F(InitializationTest, Trajectory) {
  static constexpr size_t N = 20;
  scalar_array_t tTraj(N);
  scalar_t n = 0;
  std::generate(tTraj.begin(), tTraj.end(), [&n]() mutable { return n++; });
  const vector_array_t xTraj(N, vector_t::Random(stateDim_));
  const vector_array_t uTraj(N, vector_t::Random(inputDim_));
  OperatingPoints operatingPoints(tTraj, xTraj, uTraj);

  const size_t i_0 = 1;
  const size_t i_f = 10;
  const size_t length = i_f - i_0 + 1 + 2;
  const auto t0 = tTraj[i_0] - 0.5;
  const auto tf = tTraj[i_f] + 0.5;
  operatingPoints.getSystemOperatingTrajectories(xTraj[0], t0, tf, timeTrajectory, stateTrajectory, inputTrajectory);

  ASSERT_EQ(timeTrajectory.size(), length);
  ASSERT_EQ(stateTrajectory.size(), length);
  ASSERT_EQ(inputTrajectory.size(), length);

  ASSERT_EQ(timeTrajectory.front(), t0);
  ASSERT_EQ(timeTrajectory.back(), tf);

  size_t ind = 1;
  for (size_t i = i_0; i <= i_f; i++) {
    ASSERT_TRUE(stateTrajectory[ind].isApprox(xTraj[i]));
    ASSERT_TRUE(inputTrajectory[ind].isApprox(uTraj[i]));
    ++ind;
  }
}
