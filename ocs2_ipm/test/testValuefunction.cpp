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

#include "ocs2_ipm/IpmSolver.h"

#include <ocs2_core/initialization/DefaultInitializer.h>

#include <ocs2_oc/oc_data/TimeDiscretization.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

TEST(test_valuefunction, linear_quadratic_problem) {
  constexpr int n = 3;
  constexpr int m = 2;
  constexpr int nc = 1;
  constexpr int Nsample = 10;
  constexpr ocs2::scalar_t tol = 1e-9;
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t eventTime = 1.0 / 3.0;
  const ocs2::scalar_t finalTime = 1.0;

  ocs2::OptimalControlProblem problem;

  // System
  const auto dynamics = ocs2::getRandomDynamics(n, m);
  const auto jumpMap = ocs2::matrix_t::Random(n, n);
  problem.dynamicsPtr.reset(new ocs2::LinearSystemDynamics(dynamics.dfdx, dynamics.dfdu, jumpMap));

  // Cost
  problem.costPtr->add("intermediateCost", ocs2::getOcs2Cost(ocs2::getRandomCost(n, m)));
  problem.preJumpCostPtr->add("eventCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));
  problem.finalCostPtr->add("finalCost", ocs2::getOcs2StateCost(ocs2::getRandomCost(n, 0)));

  // Reference Manager
  const ocs2::ModeSchedule modeSchedule({eventTime}, {0, 1});
  const ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Random(n)}, {ocs2::vector_t::Random(m)});
  auto referenceManagerPtr = std::make_shared<ocs2::ReferenceManager>(targetTrajectories, modeSchedule);

  problem.targetTrajectoriesPtr = &targetTrajectories;

  // Constraint
  problem.equalityConstraintPtr->add("constraint", ocs2::getOcs2Constraints(ocs2::getRandomConstraints(n, m, nc)));

  ocs2::DefaultInitializer zeroInitializer(m);

  // Solver settings
  ocs2::ipm::Settings settings;
  settings.dt = 0.05;
  settings.ipmIteration = 1;
  settings.printSolverStatistics = false;
  settings.printSolverStatus = false;
  settings.printLinesearch = false;
  settings.useFeedbackPolicy = true;
  settings.createValueFunction = true;

  // Set up solver
  ocs2::IpmSolver solver(settings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);

  // Get value function
  const ocs2::vector_t zeroState = ocs2::vector_t::Random(n);
  solver.reset();
  solver.run(startTime, zeroState, finalTime);
  const auto costToGo = solver.getValueFunction(startTime, zeroState);
  const ocs2::scalar_t zeroCost = solver.getPerformanceIndeces().cost;

  // Solve for random states and check consistency with value function
  for (int i = 0; i < Nsample; ++i) {
    const ocs2::vector_t sampleState = ocs2::vector_t::Random(n);
    solver.reset();
    solver.run(startTime, sampleState, finalTime);
    const ocs2::scalar_t sampleCost = solver.getPerformanceIndeces().cost;
    const ocs2::vector_t dx = sampleState - zeroState;

    EXPECT_NEAR(sampleCost, zeroCost + costToGo.dfdx.dot(dx) + 0.5 * dx.dot(costToGo.dfdxx * dx), tol);
  }
}
