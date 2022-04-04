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

#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

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
  static constexpr ocs2::scalar_t dt = 1e-2;

  DiscreteTranscriptionTest() {
    srand(0);

    targetTrajectories = ocs2::TargetTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});

    system = ocs2::getOcs2Dynamics(ocs2::getRandomDynamics(STATE_DIM, INPUT_DIM));

    ocs2::OptimalControlProblem problem;
    problem.dynamicsPtr.reset(system->clone());

    problem.costPtr->add("IntermediateCost", ocs2::getOcs2Cost(ocs2::getRandomCost(STATE_DIM, INPUT_DIM)));
    problem.finalCostPtr->add("FinalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(STATE_DIM, 0)));
    problem.targetTrajectoriesPtr = &targetTrajectories;

    linearization = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);

    unconstrainedLqr = ocs2::qp_solver::getLinearQuadraticApproximation(problem, linearization);

    constrainedProblem = problem;  // copies unconstrained problem
    constrainedProblem.targetTrajectoriesPtr = &targetTrajectories;

    constrainedProblem.equalityConstraintPtr->add(
        "equality", ocs2::getOcs2Constraints(ocs2::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    constrainedProblem.stateEqualityConstraintPtr->add(
        "equality", ocs2::getOcs2StateOnlyConstraints(ocs2::getRandomConstraints(STATE_DIM, 0, numStateOnlyConstraints)));
    constrainedProblem.finalEqualityConstraintPtr->add(
        "equality", ocs2::getOcs2StateOnlyConstraints(ocs2::getRandomConstraints(STATE_DIM, 0, numFinalStateOnlyConstraints)));

    constrainedLqr = ocs2::qp_solver::getLinearQuadraticApproximation(constrainedProblem, linearization);
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
      ASSERT_EQ(lqr[k].constraints.dfdu.rows(), numIntermediateConstraints);
      if (numIntermediateConstraints > 0) {
        ASSERT_EQ(lqr[k].constraints.dfdx.cols(), STATE_DIM);
        ASSERT_EQ(lqr[k].constraints.dfdu.cols(), INPUT_DIM);
      }
    }

    // Terminal Cost size
    ASSERT_EQ(lqr[N].cost.dfdxx.rows(), STATE_DIM);
    ASSERT_EQ(lqr[N].cost.dfdxx.cols(), STATE_DIM);

    // Terminal Constraint size
    ASSERT_EQ(lqr[N].constraints.f.rows(), numTerminalConstraints);
    ASSERT_EQ(lqr[N].constraints.dfdx.rows(), numTerminalConstraints);
    if (numTerminalConstraints > 0) {
      ASSERT_EQ(lqr[N].constraints.dfdx.cols(), STATE_DIM);
    }
  }

  ocs2::TargetTrajectories targetTrajectories;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
  ocs2::OptimalControlProblem constrainedProblem;
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
constexpr ocs2::scalar_t DiscreteTranscriptionTest::dt;

TEST_F(DiscreteTranscriptionTest, unconstrainedLqrHasCorrectSizes) {
  checkSizes(unconstrainedLqr, 0, 0, 0);
}

TEST_F(DiscreteTranscriptionTest, constrainedLqrHasCorrectSizes) {
  checkSizes(constrainedLqr, numStateInputConstraints, numStateOnlyConstraints, numFinalStateOnlyConstraints);
}

TEST_F(DiscreteTranscriptionTest, linearizationInvariance) {
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = linearization.timeTrajectory;

  const auto lqp2 = ocs2::qp_solver::getLinearQuadraticApproximation(constrainedProblem, linearization2);

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
