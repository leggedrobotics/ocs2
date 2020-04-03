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
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;

  using operating_points_t = ocs2::OperatingPoints<STATE_DIM, INPUT_DIM>;
  using scalar_t = operating_points_t::scalar_t;
  using scalar_array_t = operating_points_t::scalar_array_t;
  using state_vector_t = operating_points_t::state_vector_t;
  using state_vector_array_t = operating_points_t::state_vector_array_t;
  using input_vector_t = operating_points_t::input_vector_t;
  using input_vector_array_t = operating_points_t::input_vector_array_t;

  InitializationTest() = default;

  scalar_array_t timeTrajectory;
  state_vector_array_t stateTrajectory;
  input_vector_array_t inputTrajectory;
};

TEST_F(InitializationTest, SingleOperatingPoint) {
  const scalar_t t0 = 0.0;
  const scalar_t tf = 1.0;
  const state_vector_array_t xTraj = {state_vector_t::Random()};
  const input_vector_array_t uTraj = {input_vector_t::Random()};
  operating_points_t operatingPoints({t0}, xTraj, uTraj);
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
  const state_vector_array_t xTraj {state_vector_t::Random()};
  const input_vector_array_t uTraj = {input_vector_t::Random()};
  operating_points_t operatingPoints({t0}, xTraj, uTraj);
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
  std::generate(tTraj.begin(), tTraj.end(), [n = 0] () mutable { return n++; });
  const state_vector_array_t xTraj(N, state_vector_t::Random());
  const input_vector_array_t uTraj(N, input_vector_t::Random());
  operating_points_t operatingPoints(tTraj, xTraj, uTraj);

  const auto t0 = tTraj[1] - 0.5;
  const auto tf = tTraj[10] + 0.5;
  operatingPoints.getSystemOperatingTrajectories(xTraj[0], t0, tf, timeTrajectory, stateTrajectory, inputTrajectory);

  ASSERT_EQ(timeTrajectory.size(), 12);
  ASSERT_EQ(stateTrajectory.size(), 12);
  ASSERT_EQ(inputTrajectory.size(), 12);

  ASSERT_EQ(timeTrajectory.front(), t0);
  ASSERT_EQ(timeTrajectory.back(), tf);
}
