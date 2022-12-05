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

#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

#include <ocs2_qp_solver/test/testProblemsGeneration.h>

using namespace ocs2;

TEST(testContinousTimeLqr, compareWithMatlab) {
  // Set up problem with arbitrary values
  const matrix_t A = (matrix_t(2, 2) << 1.0, 2.0, 3.0, 4.0).finished();
  const matrix_t B = (matrix_t(2, 1) << 5.0, 6.0).finished();
  LinearSystemDynamics dynamics(A, B);

  const matrix_t Q = (matrix_t(2, 2) << 3.0, 2.0, 2.0, 4.0).finished();
  const matrix_t R = (matrix_t(1, 1) << 5.0).finished();
  const matrix_t P = (matrix_t(1, 2) << 0.1, 0.2).finished();
  auto cost = std::make_unique<QuadraticStateInputCost>(Q, R, P);

  const scalar_t time = 0.0;
  const vector_t state = vector_t::Zero(2);
  const vector_t input = vector_t::Zero(1);
  const TargetTrajectories targetTrajectories({time}, {state}, {input});

  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr.reset(dynamics.clone());
  problem.costPtr->add("cost", std::move(cost));
  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Solve LQR
  const auto lqrSolution = continuous_time_lqr::solve(problem, time, state, input);

  // MATLAB results, notice sign difference with MATLAB, ocs2 computes K for u = K * x
  const matrix_t K_check = (matrix_t(1, 2) << -0.905054653909129, -1.802904101100247).finished();
  const matrix_t S_check = (matrix_t(2, 2) << 1.109884545577592, -0.187358243057052, -0.187358243057052, 1.625218620131083).finished();

  const scalar_t tolerance = 1e-9;
  ASSERT_TRUE(lqrSolution.feedbackGains.isApprox(K_check, tolerance));
  ASSERT_TRUE(lqrSolution.valueFunction.isApprox(S_check, tolerance));
}

TEST(testContinousTimeLqr, evaluateCAREresidual) {
  const scalar_t careResidualNormTolerance = 1e-9;  // CARE needs to be solved up until this tolerance

  // Check random problems of increasing size.
  for (int n = 2; n < 65; n *= 2) {
    const int m = n / 2;

    // random dynamics
    const auto dynamicsMatrices = getRandomDynamics(n, m);
    const auto dynamics = getOcs2Dynamics(dynamicsMatrices);

    // random costs
    const auto costMatrices = getRandomCost(n, m);
    auto cost = getOcs2Cost(costMatrices);
    const scalar_t time = 0.0;
    const vector_t state = vector_t::Random(n);
    const vector_t input = vector_t::Random(m);
    const TargetTrajectories targetTrajectories({time}, {state}, {input});

    ocs2::OptimalControlProblem problem;
    problem.dynamicsPtr.reset(dynamics->clone());
    problem.costPtr->add("cost", std::move(cost));
    problem.targetTrajectoriesPtr = &targetTrajectories;

    // Solve LQR
    const scalar_t timeLinearization = 0.0;
    const vector_t stateLinearization = vector_t::Random(n);
    const vector_t inputLinearization = vector_t::Random(m);
    const auto lqrSolution = continuous_time_lqr::solve(problem, timeLinearization, stateLinearization, inputLinearization);

    // Check CARE
    const auto& A = dynamicsMatrices.dfdx;
    const auto& B = dynamicsMatrices.dfdu;
    const auto& Q = costMatrices.dfdxx;
    const auto& R = costMatrices.dfduu;
    const auto& P = costMatrices.dfdux;
    const auto& S = lqrSolution.valueFunction;
    const matrix_t careResidual = A.transpose() * S + S * A - (S * B + P.transpose()) * R.lu().solve(B.transpose() * S + P) + Q;
    ASSERT_LT(careResidual.norm(), careResidualNormTolerance);
  }
}
