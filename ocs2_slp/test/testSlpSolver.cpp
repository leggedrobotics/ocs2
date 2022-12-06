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

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

#include "ocs2_slp/SlpSolver.h"

namespace ocs2 {
namespace {

std::pair<PrimalSolution, std::vector<PerformanceIndex>> solve(const VectorFunctionLinearApproximation& dynamicsMatrices,
                                                               const ScalarFunctionQuadraticApproximation& costMatrices,
                                                               const ocs2::scalar_t tol) {
  int n = dynamicsMatrices.dfdu.rows();
  int m = dynamicsMatrices.dfdu.cols();

  ocs2::OptimalControlProblem problem;

  // System
  problem.dynamicsPtr = getOcs2Dynamics(dynamicsMatrices);

  // Cost
  problem.costPtr->add("intermediateCost", ocs2::getOcs2Cost(costMatrices));
  problem.finalCostPtr->add("finalCost", ocs2::getOcs2StateCost(costMatrices));

  // Reference Manager
  ocs2::TargetTrajectories targetTrajectories({0.0}, {ocs2::vector_t::Ones(n)}, {ocs2::vector_t::Ones(m)});
  std::shared_ptr<ReferenceManager> referenceManagerPtr(new ReferenceManager(targetTrajectories));

  problem.targetTrajectoriesPtr = &referenceManagerPtr->getTargetTrajectories();

  problem.equalityConstraintPtr->add("intermediateCost", ocs2::getOcs2Constraints(getRandomConstraints(n, m, 0)));

  ocs2::DefaultInitializer zeroInitializer(m);

  // Solver settings
  auto getPipgSettings = [&]() {
    ocs2::pipg::Settings settings;
    settings.maxNumIterations = 30000;
    settings.absoluteTolerance = tol;
    settings.relativeTolerance = 1e-2;
    settings.lowerBoundH = 1e-3;
    settings.checkTerminationInterval = 1;
    settings.displayShortSummary = true;
    return settings;
  };

  const auto slpSettings = [&]() {
    ocs2::slp::Settings settings;
    settings.dt = 0.05;
    settings.slpIteration = 10;
    settings.scalingIteration = 3;
    settings.printSolverStatistics = true;
    settings.printSolverStatus = true;
    settings.printLinesearch = true;
    settings.nThreads = 100;
    settings.pipgSettings = getPipgSettings();
    return settings;
  }();

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = ocs2::vector_t::Ones(n);

  // Construct solver
  ocs2::SlpSolver solver(slpSettings, problem, zeroInitializer);
  solver.setReferenceManager(referenceManagerPtr);

  // Solve
  solver.run(startTime, initState, finalTime);
  return {solver.primalSolution(finalTime), solver.getIterationsLog()};
}

}  // namespace
}  // namespace ocs2

TEST(testSlpSolver, test_unconstrained) {
  int n = 3;
  int m = 2;
  const double tol = 1e-9;
  const auto dynamics = ocs2::getRandomDynamics(n, m);
  const auto costs = ocs2::getRandomCost(n, m);
  const auto result = ocs2::solve(dynamics, costs, tol);

  /*
   * Assert performance
   * - Contains 2 performance indices, 1 for the initialization, 1 for the iteration.
   * - Linear dynamics should be satisfied after the step.
   */
  ASSERT_LE(result.second.size(), 2);
  ASSERT_LT(result.second.back().dynamicsViolationSSE, tol);
}
