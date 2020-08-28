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

#include "ocs2_qp_solver/QpDiscreteTranscription.h"
#include "ocs2_qp_solver/test/testProblemsGeneration.h"

class DiscreteTranscriptionTest : public testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr ocs2::scalar_t dt = 1e-2;

  DiscreteTranscriptionTest() {
    srand(0);
    cost = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM));
    costDesiredTrajectories =
        ocs2::CostDesiredTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});
    cost->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);
    system = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    linearization = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    lqp = ocs2::qp_solver::getLinearQuadraticApproximation(*cost, *system, linearization);
  }

  std::unique_ptr<ocs2::CostFunctionBase> cost;
  ocs2::CostDesiredTrajectories costDesiredTrajectories;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
  ocs2::qp_solver::ContinuousTrajectory linearization;
  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqp;
};

constexpr size_t DiscreteTranscriptionTest::N;
constexpr size_t DiscreteTranscriptionTest::STATE_DIM;
constexpr size_t DiscreteTranscriptionTest::INPUT_DIM;
constexpr ocs2::scalar_t DiscreteTranscriptionTest::dt;

TEST_F(DiscreteTranscriptionTest, approximationHasCorrectSizes) {
  const auto n = STATE_DIM;
  const auto m = INPUT_DIM;
  ASSERT_EQ(lqp.size(), N + 1);
  for (int k = 0; k < N; ++k) {
    // Cost sizes
    ASSERT_EQ(lqp[k].cost.dfdxx.rows(), n);
    ASSERT_EQ(lqp[k].cost.dfdxx.cols(), n);
    ASSERT_EQ(lqp[k].cost.dfdux.rows(), m);
    ASSERT_EQ(lqp[k].cost.dfdux.cols(), n);
    ASSERT_EQ(lqp[k].cost.dfduu.rows(), m);
    ASSERT_EQ(lqp[k].cost.dfduu.cols(), m);

    // Dynamics sizes
    ASSERT_EQ(lqp[k].dynamics.dfdx.rows(), n);
    ASSERT_EQ(lqp[k].dynamics.dfdx.cols(), n);
    ASSERT_EQ(lqp[k].dynamics.dfdu.rows(), n);
    ASSERT_EQ(lqp[k].dynamics.dfdu.cols(), m);
  }

  // Terminal Cost size
  ASSERT_EQ(lqp[N].cost.dfdxx.rows(), n);
  ASSERT_EQ(lqp[N].cost.dfdxx.cols(), n);
}

TEST_F(DiscreteTranscriptionTest, linearizationInvariance) {
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 = ocs2::qp_solver::getLinearQuadraticApproximation(*cost, *system, linearization2);

  // All matrices should stay the same. The linear and constant parts changes
  for (int k = 0; k < N; ++k) {
    // Cost
    ASSERT_TRUE(lqp[k].cost.dfdxx.isApprox(lqp2[k].cost.dfdxx));
    ASSERT_TRUE(lqp[k].cost.dfdux.isApprox(lqp2[k].cost.dfdux));
    ASSERT_TRUE(lqp[k].cost.dfduu.isApprox(lqp2[k].cost.dfduu));

    // Dynamics
    ASSERT_TRUE(lqp[k].dynamics.dfdx.isApprox(lqp2[k].dynamics.dfdx));
    ASSERT_TRUE(lqp[k].dynamics.dfdu.isApprox(lqp2[k].dynamics.dfdu));
  }

  // Terminal Cost size
  ASSERT_TRUE(lqp[N].cost.dfdxx.isApprox(lqp2[N].cost.dfdxx));
}
