//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/QpDiscreteTranscription.h"
#include "ocs2_qp_solver/test/testProblemsGeneration.h"

class DiscreteTranscriptionTest : public testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr size_t numStateInputConstraints = 1;
  static constexpr size_t numStateOnlyConstraints = 1;
  static constexpr size_t numFinalStateOnlyConstraints = 1;
  using SystemDynamics_t = ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
  using costFunction_t = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using input_vector_t = costFunction_t::input_vector_t;
  using state_vector_t = costFunction_t::state_vector_t;

  DiscreteTranscriptionTest() {
    srand(0);
    const auto cost = ocs2::qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(
        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
        state_vector_t::Random(), input_vector_t::Random(), state_vector_t::Random());
    costWrapper.reset(new ocs2::qp_solver::CostWrapper(*cost));
    const auto system = ocs2::qp_solver::getOcs2Dynamics<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    systemWrapper.reset(new ocs2::qp_solver::SystemWrapper(*system));
    const auto constraint = ocs2::qp_solver::getOcs2Constraints<STATE_DIM, INPUT_DIM>(
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateOnlyConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numFinalStateOnlyConstraints));
    constraintsWrapper.reset(new ocs2::qp_solver::ConstraintsWrapper(*constraint));
    linearization = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
    unconstrainedLqr = ocs2::qp_solver::getLinearQuadraticApproximation(*costWrapper, *systemWrapper, nullptr, linearization);
    constrainedLqr =
        ocs2::qp_solver::getLinearQuadraticApproximation(*costWrapper, *systemWrapper, constraintsWrapper.get(), linearization);
  }

  void checkSizes(const std::vector<ocs2::qp_solver::LinearQuadraticStage>& lqr, size_t numStateInputConstraints,
                  size_t numStateOnlyConstraints, size_t numTerminalConstraints) const {
    ASSERT_EQ(lqr.size(), N + 1);
    for (int k = 0; k < N; ++k) {
      // Cost sizes
      ASSERT_EQ(lqr[k].cost.dfdxx.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfdxx.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfdux.rows(), INPUT_DIM);
      ASSERT_EQ(lqr[k].cost.dfdux.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].cost.dfduu.rows(), INPUT_DIM);
      ASSERT_EQ(lqr[k].cost.dfduu.cols(), INPUT_DIM);

      // Dynamics sizes
      ASSERT_EQ(lqr[k].dynamics.dfdx.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdx.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdu.rows(), STATE_DIM);
      ASSERT_EQ(lqr[k].dynamics.dfdu.cols(), INPUT_DIM);

      // Constraint sizes
      const auto numIntermediateConstraints = k == 0 ? numStateInputConstraints : numStateInputConstraints + numStateOnlyConstraints;
      ASSERT_EQ(lqr[k].constraints.f.rows(), numIntermediateConstraints);
      ASSERT_EQ(lqr[k].constraints.dfdx.rows(), numIntermediateConstraints);
      ASSERT_EQ(lqr[k].constraints.dfdx.cols(), STATE_DIM);
      ASSERT_EQ(lqr[k].constraints.dfdu.rows(), numIntermediateConstraints);
      ASSERT_EQ(lqr[k].constraints.dfdu.cols(), INPUT_DIM);
    }

    // Terminal Cost size
    ASSERT_EQ(lqr[N].cost.dfdxx.rows(), STATE_DIM);
    ASSERT_EQ(lqr[N].cost.dfdxx.cols(), STATE_DIM);

    // Terminal Constraint size
    ASSERT_EQ(lqr[N].constraints.f.rows(), numTerminalConstraints);
    ASSERT_EQ(lqr[N].constraints.dfdx.rows(), numTerminalConstraints);
    ASSERT_EQ(lqr[N].constraints.dfdx.cols(), STATE_DIM);
  }

  std::unique_ptr<ocs2::qp_solver::CostWrapper> costWrapper;
  std::unique_ptr<ocs2::qp_solver::SystemWrapper> systemWrapper;
  std::unique_ptr<ocs2::qp_solver::ConstraintsWrapper> constraintsWrapper;
  ocs2::qp_solver::ContinuousTrajectory linearization;
  std::vector<ocs2::qp_solver::LinearQuadraticStage> unconstrainedLqr;
  std::vector<ocs2::qp_solver::LinearQuadraticStage> constrainedLqr;
};

constexpr size_t DiscreteTranscriptionTest::N;
constexpr size_t DiscreteTranscriptionTest::STATE_DIM;
constexpr size_t DiscreteTranscriptionTest::INPUT_DIM;
constexpr size_t DiscreteTranscriptionTest::numStateInputConstraints;
constexpr size_t DiscreteTranscriptionTest::numStateOnlyConstraints;
constexpr size_t DiscreteTranscriptionTest::numFinalStateOnlyConstraints;

TEST_F(DiscreteTranscriptionTest, unconstrainedLqrHasCorrectSizes) {
  checkSizes(unconstrainedLqr, 0, 0, 0);
}

TEST_F(DiscreteTranscriptionTest, constrainedLqrHasCorrectSizes) {
  checkSizes(constrainedLqr, numStateInputConstraints, numStateOnlyConstraints, numFinalStateOnlyConstraints);
}

TEST_F(DiscreteTranscriptionTest, linearizationInvariance) {
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 =
      ocs2::qp_solver::getLinearQuadraticApproximation(*costWrapper, *systemWrapper, constraintsWrapper.get(), linearization2);

  // All matrices should stay the same. The linear and constant parts changes
  for (int k = 0; k < N; ++k) {
    // Cost
    ASSERT_TRUE(constrainedLqr[k].cost.dfdxx.isApprox(lqp2[k].cost.dfdxx));
    ASSERT_TRUE(constrainedLqr[k].cost.dfdux.isApprox(lqp2[k].cost.dfdux));
    ASSERT_TRUE(constrainedLqr[k].cost.dfduu.isApprox(lqp2[k].cost.dfduu));

    // Dynamics
    ASSERT_TRUE(constrainedLqr[k].dynamics.dfdx.isApprox(lqp2[k].dynamics.dfdx));
    ASSERT_TRUE(constrainedLqr[k].dynamics.dfdu.isApprox(lqp2[k].dynamics.dfdu));

    // Constraints
    ASSERT_TRUE(constrainedLqr[k].constraints.dfdx.isApprox(lqp2[k].constraints.dfdx));
    ASSERT_TRUE(constrainedLqr[k].constraints.dfdu.isApprox(lqp2[k].constraints.dfdu));
  }

  // Terminal Cost
  ASSERT_TRUE(constrainedLqr[N].cost.dfdxx.isApprox(lqp2[N].cost.dfdxx));

  // Terminal Constraints
  ASSERT_TRUE(constrainedLqr[N].constraints.dfdx.isApprox(lqp2[N].constraints.dfdx));
}
