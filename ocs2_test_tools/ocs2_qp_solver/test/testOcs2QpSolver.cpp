//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

#include "ocs2_qp_solver/Ocs2QpSolver.h"
#include "ocs2_qp_solver/test/testProblemsGeneration.h"

class Ocs2QpSolverTest : public testing::Test {
 protected:
  static constexpr size_t N = 10;  // Trajectory length
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  static constexpr ocs2::scalar_t dt = 1e-3;

  Ocs2QpSolverTest() {
    srand(0);
    cost = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), ocs2::vector_t::Random(STATE_DIM),
                                        ocs2::vector_t::Random(INPUT_DIM), ocs2::vector_t::Random(STATE_DIM));
    system = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    x0 = ocs2::vector_t::Random(STATE_DIM);
    solution = solveLinearQuadraticOptimalControlProblem(*cost, *system, nominalTrajectory, x0);
  }

  std::unique_ptr<ocs2::CostFunctionBase> cost;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  ocs2::vector_t x0;
  ocs2::qp_solver::ContinuousTrajectory solution;
};

constexpr size_t Ocs2QpSolverTest::N;
constexpr size_t Ocs2QpSolverTest::STATE_DIM;
constexpr size_t Ocs2QpSolverTest::INPUT_DIM;
constexpr ocs2::scalar_t Ocs2QpSolverTest::dt;

TEST_F(Ocs2QpSolverTest, initialCondition) {
  ASSERT_TRUE(x0.isApprox(solution.stateTrajectory.front()));
}

TEST_F(Ocs2QpSolverTest, satisfiesDynamics) {
  // Forward integrate with solution u(t) and check x(t)
  ocs2::vector_t x = x0;
  for (int k = 0; k < N; ++k) {
    auto dt = solution.timeTrajectory[k + 1] - solution.timeTrajectory[k];
    x += dt * system->computeFlowMap(solution.timeTrajectory[k], x, solution.inputTrajectory[k]);
    ASSERT_TRUE(x.isApprox(solution.stateTrajectory[k + 1]));
  }
}

TEST_F(Ocs2QpSolverTest, invariantUnderLinearization) {
  // Different nominalTrajectory, with same time discretization
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = nominalTrajectory.timeTrajectory;

  // Compare solutions
  auto solution2 = solveLinearQuadraticOptimalControlProblem(*cost, *system, linearization2, x0);
  ASSERT_TRUE(ocs2::qp_solver::isEqual(solution.stateTrajectory, solution2.stateTrajectory));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(solution.inputTrajectory, solution2.inputTrajectory));
}

TEST_F(Ocs2QpSolverTest, knownSolutionAtOrigin) {
  // If the cost's nominal trajectory is set to zero, and the initial state is zero, then the solution has only zeros.
  const auto zeroCost = ocs2::qp_solver::getOcs2Cost(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                     ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), ocs2::vector_t::Zero(STATE_DIM),
                                                     ocs2::vector_t::Zero(INPUT_DIM), ocs2::vector_t::Zero(STATE_DIM));
  const auto zeroX0 = ocs2::vector_t::Zero(STATE_DIM);

  // Obtain solution, with non-zero nominalTrajectory
  auto zeroSolution = solveLinearQuadraticOptimalControlProblem(*zeroCost, *system, nominalTrajectory, zeroX0);

  ocs2::vector_array_t allStatesZero(N + 1, ocs2::vector_t::Zero(STATE_DIM));
  ocs2::vector_array_t allInputsZero(N, ocs2::vector_t::Zero(INPUT_DIM));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.stateTrajectory, allStatesZero));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.inputTrajectory, allInputsZero));
}
