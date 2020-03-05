//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include <ocs2_qp_solver/QpDiscreteTranscription.h>

#include "testProblemsGeneration.h"

class DiscreteTranscriptionTest : public testing::Test {
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
    const auto cost = ocs2::qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                             ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), state_vector_t::Random(),
                                                             input_vector_t::Random(), state_vector_t::Random());
    costWrapper.reset(new ocs2::qp_solver::CostWrapper(*cost));
    const auto system = ocs2::qp_solver::getOcs2Dynamics<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    systemWrapper.reset(new ocs2::qp_solver::SystemWrapper(*system));
    linearization = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
    lqp = ocs2::qp_solver::getLinearQuadraticApproximation(*costWrapper, *systemWrapper, linearization);
  }

  std::unique_ptr<ocs2::qp_solver::CostWrapper> costWrapper;
  std::unique_ptr<ocs2::qp_solver::SystemWrapper> systemWrapper;
  ocs2::qp_solver::ContinuousTrajectory linearization;
  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqp;
};

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
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 = ocs2::qp_solver::getLinearQuadraticApproximation(*costWrapper, *systemWrapper, linearization2);

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