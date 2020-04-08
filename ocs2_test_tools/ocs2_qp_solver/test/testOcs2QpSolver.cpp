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
  static constexpr size_t STATE_DIM = 5;
  static constexpr size_t INPUT_DIM = 3;
  static constexpr size_t numStateInputConstraints = 1;
  static constexpr size_t numStateOnlyConstraints = 1;
  static constexpr size_t numFinalStateOnlyConstraints = 2;
  static constexpr ocs2::scalar_t precision = 1e-9;
  static constexpr ocs2::scalar_t dt = 1e-3;
  using SystemDynamics_t = ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
  using costFunction_t = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using constraint_t = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;
  using input_vector_t = costFunction_t::input_vector_t;
  using state_vector_t = costFunction_t::state_vector_t;

  Ocs2QpSolverTest() {
    srand(0);
    cost = ocs2::qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                              ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                              state_vector_t::Random(), input_vector_t::Random(), state_vector_t::Random());
    system = ocs2::qp_solver::getOcs2Dynamics<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    constraint = ocs2::qp_solver::getOcs2Constraints<STATE_DIM, INPUT_DIM>(
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateOnlyConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, numFinalStateOnlyConstraints));
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    x0 = state_vector_t::Random();
    constrainedSolution = solveLinearQuadraticOptimalControlProblem(*cost, *system, *constraint, nominalTrajectory, x0);
    unconstrainedSolution = solveLinearQuadraticOptimalControlProblem(*cost, *system, nominalTrajectory, x0);
  }

  void checkDyanmics(const ocs2::qp_solver::ContinuousTrajectory& solution) const {
    // Forward integrate with unconstrainedSolution u(t) and check x(t)
    state_vector_t x = x0;
    ocs2::qp_solver::SystemWrapper systemWrapper(*system);
    for (int k = 0; k < N; ++k) {
      auto dt = solution.timeTrajectory[k + 1] - solution.timeTrajectory[k];
      x += dt * systemWrapper.getFlowMap(solution.timeTrajectory[k], x, solution.inputTrajectory[k]);
      ASSERT_TRUE(x.isApprox(solution.stateTrajectory[k + 1], precision));
    }
  }

  std::unique_ptr<costFunction_t> cost;
  std::unique_ptr<SystemDynamics_t> system;
  std::unique_ptr<constraint_t> constraint;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  state_vector_t x0;
  ocs2::qp_solver::ContinuousTrajectory constrainedSolution;
  ocs2::qp_solver::ContinuousTrajectory unconstrainedSolution;
};

constexpr size_t Ocs2QpSolverTest::N;
constexpr size_t Ocs2QpSolverTest::STATE_DIM;
constexpr size_t Ocs2QpSolverTest::INPUT_DIM;
constexpr size_t Ocs2QpSolverTest::numStateInputConstraints;
constexpr size_t Ocs2QpSolverTest::numStateOnlyConstraints;
constexpr size_t Ocs2QpSolverTest::numFinalStateOnlyConstraints;
constexpr ocs2::scalar_t Ocs2QpSolverTest::precision;
constexpr ocs2::scalar_t Ocs2QpSolverTest::dt;

TEST_F(Ocs2QpSolverTest, initialCondition) {
  ASSERT_TRUE(x0.isApprox(constrainedSolution.stateTrajectory.front(), precision));
  ASSERT_TRUE(x0.isApprox(unconstrainedSolution.stateTrajectory.front(), precision));
}

TEST_F(Ocs2QpSolverTest, satisfiesDynamics) {
  checkDyanmics(constrainedSolution);
  checkDyanmics(unconstrainedSolution);
}

TEST_F(Ocs2QpSolverTest, satisfiesConstraints) {
  ocs2::qp_solver::ConstraintsWrapper constraintsWrapper(*constraint);
  const auto g0 = constraintsWrapper.getInitialConstraint(constrainedSolution.timeTrajectory[0], constrainedSolution.stateTrajectory[0],
                                                          constrainedSolution.inputTrajectory[0]);
  ASSERT_TRUE(g0.isZero(precision));
  for (int k = 1; k < N; ++k) {
    const auto g = constraintsWrapper.getConstraint(constrainedSolution.timeTrajectory[k], constrainedSolution.stateTrajectory[k],
                                                    constrainedSolution.inputTrajectory[k]);
    ASSERT_TRUE(g.isZero(precision));
  }
  const auto gf = constraintsWrapper.getTerminalConstraint(constrainedSolution.timeTrajectory[N], constrainedSolution.stateTrajectory[N]);
  ASSERT_TRUE(gf.isZero(precision));
}

TEST_F(Ocs2QpSolverTest, invariantUnderLinearization) {
  // Different nominalTrajectory, with same time discretization
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = nominalTrajectory.timeTrajectory;

  // Compare solutions
  auto solution2 = solveLinearQuadraticOptimalControlProblem(*cost, *system, *constraint, linearization2, x0);
  ASSERT_TRUE(ocs2::qp_solver::isEqual(constrainedSolution.stateTrajectory, solution2.stateTrajectory, precision));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(constrainedSolution.inputTrajectory, solution2.inputTrajectory, precision));
}

TEST_F(Ocs2QpSolverTest, knownSolutionAtOrigin) {
  // If the cost's nominal trajectory is set to zero, and the initial state is zero, then the unconstrained solution has only zeros.
  const auto zeroCost = ocs2::qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(
      ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM), state_vector_t::Zero(),
      input_vector_t::Zero(), state_vector_t::Zero());
  const auto zeroX0 = state_vector_t::Zero();

  // Obtain solution, with non-zero nominalTrajectory
  auto zeroSolution = solveLinearQuadraticOptimalControlProblem(*zeroCost, *system, nominalTrajectory, zeroX0);

  ocs2::dynamic_vector_array_t allStatesZero(N + 1, ocs2::dynamic_vector_t::Zero(STATE_DIM));
  ocs2::dynamic_vector_array_t allInputsZero(N, ocs2::dynamic_vector_t::Zero(INPUT_DIM));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.stateTrajectory, allStatesZero));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.inputTrajectory, allInputsZero));
}
