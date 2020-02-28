//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include <ocs2_qp_solver/QpDiscreteTranscription.h>

#include "testProblemsGeneration.h"

class DiscreteTranscriptionTest : public ::testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  using SystemDynamics_t = ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
  using costFunction_t = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using input_vector_t = costFunction_t::input_vector_t;
  using state_vector_t = costFunction_t::state_vector_t;

  DiscreteTranscriptionTest() {
    srand(0);
    cost = ocs2_qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(ocs2_qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                             ocs2_qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), state_vector_t::Random(),
                                                             input_vector_t::Random(), state_vector_t::Random());
    system = ocs2_qp_solver::getOcs2Dynamics<STATE_DIM, INPUT_DIM>(ocs2_qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    linearization = ocs2_qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
    lqp = ocs2_qp_solver::getLinearQuadraticApproximation(*cost, *system, linearization);
  }

  std::unique_ptr<costFunction_t> cost;
  std::unique_ptr<SystemDynamics_t> system;
  ocs2_qp_solver::ContinuousTrajectory linearization;
  std::vector<ocs2_qp_solver::LinearQuadraticStage> lqp;
};

TEST_F(DiscreteTranscriptionTest, approximationHasCorrectSizes) {
  const auto n = STATE_DIM;
  const auto m = INPUT_DIM;
  ASSERT_EQ(lqp.size(), N + 1);
  for (int k = 0; k < N; ++k) {
    // Cost sizes
    ASSERT_EQ(lqp[k].cost.Q.rows(), n);
    ASSERT_EQ(lqp[k].cost.Q.cols(), n);
    ASSERT_EQ(lqp[k].cost.P.rows(), m);
    ASSERT_EQ(lqp[k].cost.P.cols(), n);
    ASSERT_EQ(lqp[k].cost.R.rows(), m);
    ASSERT_EQ(lqp[k].cost.R.cols(), m);

    // Dynamics sizes
    ASSERT_EQ(lqp[k].dynamics.A.rows(), n);
    ASSERT_EQ(lqp[k].dynamics.A.cols(), n);
    ASSERT_EQ(lqp[k].dynamics.B.rows(), n);
    ASSERT_EQ(lqp[k].dynamics.B.cols(), m);
  }

  // Terminal Cost size
  ASSERT_EQ(lqp[N].cost.Q.rows(), n);
  ASSERT_EQ(lqp[N].cost.Q.cols(), n);
}

TEST_F(DiscreteTranscriptionTest, linearizationInvariance) {
  auto linearization2 = ocs2_qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 = ocs2_qp_solver::getLinearQuadraticApproximation(*cost, *system, linearization2);

  // All matrices should stay the same. The linear and constant parts changes
  for (int k = 0; k < N; ++k) {
    // Cost
    ASSERT_TRUE(lqp[k].cost.Q.isApprox(lqp2[k].cost.Q));
    ASSERT_TRUE(lqp[k].cost.P.isApprox(lqp2[k].cost.P));
    ASSERT_TRUE(lqp[k].cost.R.isApprox(lqp2[k].cost.R));

    // Dynamics
    ASSERT_TRUE(lqp[k].dynamics.A.isApprox(lqp2[k].dynamics.A));
    ASSERT_TRUE(lqp[k].dynamics.B.isApprox(lqp2[k].dynamics.B));
  }

  // Terminal Cost size
  ASSERT_TRUE(lqp[N].cost.Q.isApprox(lqp2[N].cost.Q));
}