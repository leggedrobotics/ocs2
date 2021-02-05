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

#include "ocs2_ddp/ContinuousTimeLqr.h"

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

using namespace ocs2;

TEST(testContinousTimeLqr, compareWithMatlab) {
  // Set up problem with arbitrary values
  matrix_t A(2, 2);
  A << 1.0, 2.0, 3.0, 4.0;
  matrix_t B(2, 1);
  B << 5.0, 6.0;
  LinearSystemDynamics dynamics(A, B);

  matrix_t Q(2, 2);
  Q << 3.0, 2.0, 2.0, 4.0;
  matrix_t R(1, 1);
  R << 5.0;
  matrix_t P(1, 2);
  P << 0.1, 0.2;
  matrix_t QFinal = matrix_t::Zero(2, 2);
  QuadraticCostFunction cost(Q, R, QFinal, P);

  scalar_t time = 0.0;
  vector_t state = vector_t::Zero(2);
  vector_t input = vector_t::Zero(1);
  CostDesiredTrajectories costDesiredTrajectories({time}, {state}, {input});
  cost.setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);

  // Solve LQR
  auto lqrSolution = continuous_time_lqr::solve(dynamics, cost, time, state, input);

  // MATLAB results
  matrix_t K_check(1, 2);
  K_check << -0.905054653909129, -1.802904101100247;  // Matlab returns positive gains, ocs2 computes the negative gains.
  matrix_t S_check(2, 2);
  S_check << 1.109884545577592, -0.187358243057052, -0.187358243057052, 1.625218620131083;

  const scalar_t tolerance = 1e-9;
  ASSERT_TRUE(lqrSolution.feedbackGains.isApprox(K_check, tolerance));
  ASSERT_TRUE(lqrSolution.valueFunction.isApprox(S_check, tolerance));
}
