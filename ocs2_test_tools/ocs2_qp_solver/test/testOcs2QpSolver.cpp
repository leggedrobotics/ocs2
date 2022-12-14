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

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/model_data/Metrics.h>

#include <ocs2_core/test/testTools.h>

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

  void checkDynamics(const ocs2::qp_solver::ContinuousTrajectory& solution) const {
    // Forward integrate with unconstrainedSolution u(t) and check x(t)
    ocs2::vector_t x = x0;
    for (int k = 0; k < N; ++k) {
      auto dt = solution.timeTrajectory[k + 1] - solution.timeTrajectory[k];
      x += dt * system->computeFlowMap(solution.timeTrajectory[k], x, solution.inputTrajectory[k]);
      ASSERT_TRUE(x.isApprox(solution.stateTrajectory[k + 1], precision));
    }
  }

  Ocs2QpSolverTest() {
    srand(0);

    system = ocs2::getOcs2Dynamics(ocs2::getRandomDynamics(STATE_DIM, INPUT_DIM));

    // create unconstrained problem
    unconstrainedProblem = ocs2::OptimalControlProblem();  // reset
    unconstrainedProblem.dynamicsPtr.reset(system->clone());

    unconstrainedProblem.costPtr->add("IntermediateCost", ocs2::getOcs2Cost(ocs2::getRandomCost(STATE_DIM, INPUT_DIM)));
    unconstrainedProblem.finalCostPtr->add("FinalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(STATE_DIM, 0)));

    // create constrained problem
    constrainedProblem = unconstrainedProblem;  // copies unconstrained problem

    constrainedProblem.equalityConstraintPtr->add(
        "equality", ocs2::getOcs2Constraints(ocs2::getRandomConstraints(STATE_DIM, INPUT_DIM, numStateInputConstraints)));
    constrainedProblem.stateEqualityConstraintPtr->add(
        "equality", ocs2::getOcs2StateOnlyConstraints(ocs2::getRandomConstraints(STATE_DIM, 0, numStateOnlyConstraints)));
    constrainedProblem.finalEqualityConstraintPtr->add(
        "equality", ocs2::getOcs2StateOnlyConstraints(ocs2::getRandomConstraints(STATE_DIM, 0, numFinalStateOnlyConstraints)));

    targetTrajectories = ocs2::TargetTrajectories({0.0}, {ocs2::vector_t::Random(STATE_DIM)}, {ocs2::vector_t::Random(INPUT_DIM)});
    constrainedProblem.targetTrajectoriesPtr = &targetTrajectories;
    unconstrainedProblem.targetTrajectoriesPtr = &targetTrajectories;

    nominalTrajectory = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
    x0 = ocs2::vector_t::Random(STATE_DIM);
    unconstrainedSolution = solveLinearQuadraticOptimalControlProblem(unconstrainedProblem, nominalTrajectory, x0);
    constrainedSolution = solveLinearQuadraticOptimalControlProblem(constrainedProblem, nominalTrajectory, x0);
  }

  ocs2::TargetTrajectories targetTrajectories;
  std::unique_ptr<ocs2::SystemDynamicsBase> system;
  ocs2::OptimalControlProblem unconstrainedProblem;
  ocs2::OptimalControlProblem constrainedProblem;
  ocs2::qp_solver::ContinuousTrajectory nominalTrajectory;
  ocs2::vector_t x0;
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
  checkDynamics(constrainedSolution);
  checkDynamics(unconstrainedSolution);
}

TEST_F(Ocs2QpSolverTest, satisfiesConstraints) {
  auto& preComputation = *unconstrainedProblem.preComputationPtr;
  const auto t0 = constrainedSolution.timeTrajectory[0];
  const auto& x0 = constrainedSolution.stateTrajectory[0];
  const auto& u0 = constrainedSolution.inputTrajectory[0];
  preComputation.request(ocs2::Request::Constraint, t0, x0, u0);

  const auto g0 = ocs2::toVector(unconstrainedProblem.equalityConstraintPtr->getValue(t0, x0, u0, preComputation));
  ASSERT_TRUE(g0.isZero(precision));

  for (int k = 1; k < N; ++k) {
    const auto t = constrainedSolution.timeTrajectory[k];
    const auto& x = constrainedSolution.stateTrajectory[k];
    const auto& u = constrainedSolution.inputTrajectory[k];
    preComputation.request(ocs2::Request::Constraint, t, x, u);

    const auto g = ocs2::toVector(unconstrainedProblem.equalityConstraintPtr->getValue(t, x, u, preComputation));
    ASSERT_TRUE(g.isZero(precision));

    const auto h = ocs2::toVector(unconstrainedProblem.stateEqualityConstraintPtr->getValue(t, x, preComputation));
    ASSERT_TRUE(h.isZero(precision));
  }

  const auto tf = constrainedSolution.timeTrajectory[N];
  const auto& xf = constrainedSolution.stateTrajectory[N];
  preComputation.requestFinal(ocs2::Request::Constraint, tf, xf);

  const auto gf = ocs2::toVector(unconstrainedProblem.finalEqualityConstraintPtr->getValue(tf, xf, preComputation));
  ASSERT_TRUE(gf.isZero(precision));
}

TEST_F(Ocs2QpSolverTest, invariantUnderLinearization) {
  // Different nominalTrajectory, with same time discretization
  auto linearization2 = ocs2::qp_solver::getRandomTrajectory(N, STATE_DIM, INPUT_DIM, dt);
  linearization2.timeTrajectory = nominalTrajectory.timeTrajectory;

  // Compare solutions
  auto solution2 = solveLinearQuadraticOptimalControlProblem(constrainedProblem, linearization2, x0);
  ASSERT_TRUE(ocs2::isEqual(constrainedSolution.stateTrajectory, solution2.stateTrajectory, precision));
  ASSERT_TRUE(ocs2::isEqual(constrainedSolution.inputTrajectory, solution2.inputTrajectory, precision));
}

TEST_F(Ocs2QpSolverTest, knownSolutionAtOrigin) {
  // If the cost's nominal trajectory is set to zero, and the initial state is zero, then the solution has only zeros.
  targetTrajectories = ocs2::TargetTrajectories({0.0}, {ocs2::vector_t::Zero(STATE_DIM)}, {ocs2::vector_t::Zero(INPUT_DIM)});
  unconstrainedProblem.targetTrajectoriesPtr = &targetTrajectories;
  const auto zeroX0 = ocs2::vector_t::Zero(STATE_DIM);

  // Obtain solution, with non-zero nominalTrajectory
  auto zeroSolution = solveLinearQuadraticOptimalControlProblem(unconstrainedProblem, nominalTrajectory, zeroX0);

  ocs2::vector_array_t allStatesZero(N + 1, ocs2::vector_t::Zero(STATE_DIM));
  ocs2::vector_array_t allInputsZero(N, ocs2::vector_t::Zero(INPUT_DIM));
  ASSERT_TRUE(ocs2::isEqual(zeroSolution.stateTrajectory, allStatesZero));
  ASSERT_TRUE(ocs2::isEqual(zeroSolution.inputTrajectory, allInputsZero));
}
