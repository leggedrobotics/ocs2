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
                                        ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM));
    costDesiredTrajectories =
        ocs2::CostDesiredTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});
    cost->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);
    system = ocs2::qp_solver::getOcs2Dynamics(ocs2::qp_solver::getRandomDynamics(STATE_DIM, INPUT_DIM));
    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    x0 = ocs2::vector_t::Random(STATE_DIM);
    solution = solveLinearQuadraticOptimalControlProblem(*cost, *system, nominalTrajectory, x0);
  }

  std::unique_ptr<ocs2::CostFunctionBase> cost;
  ocs2::CostDesiredTrajectories costDesiredTrajectories;
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
  costDesiredTrajectories = ocs2::CostDesiredTrajectories({0.0}, {ocs2::vector_t::Zero(STATE_DIM)}, {ocs2::vector_t::Zero(INPUT_DIM)});
  cost->setCostDesiredTrajectoriesPtr(&costDesiredTrajectories);
  const auto zeroX0 = ocs2::vector_t::Zero(STATE_DIM);

  // Obtain solution, with non-zero nominalTrajectory
  auto zeroSolution = solveLinearQuadraticOptimalControlProblem(*cost, *system, nominalTrajectory, zeroX0);

  ocs2::vector_array_t allStatesZero(N + 1, ocs2::vector_t::Zero(STATE_DIM));
  ocs2::vector_array_t allInputsZero(N, ocs2::vector_t::Zero(INPUT_DIM));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.stateTrajectory, allStatesZero));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(zeroSolution.inputTrajectory, allInputsZero));
}
