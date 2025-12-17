/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/synchronized_module/SolverObserver.h>
#include <ocs2_oc/test/dynamics_hybrid_slq_test.h>

#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

#include <ocs2_ddp/SLQ.h>

/*
 * Test for StateTriggeredRollout in combination with SLQ
 *
 * The system being tested is a combination of two Linear system
 * it switches between these systems on an event
 *
 * Guard Surfaces are:   x[0]*x[1] < 0 when in mode 0
 *                       x[0]*x[1] > 0 when in mode 1
 *
 * Cost function is:     x(t)^T Q x(t) + u(t)^T R u(t) + x(t1)^T P x(t1)^T
 *                       Q = P = [50,0;0,50];
 *                       R = 1;
 *
 * Constraints are:      |u|< 2
 *                       |x[0]| < 2
 *
 * The following tests are implemented and performed:
 * (1) No penetration of Guard Surfaces
 * (2) Constraint compliance
 * (3) Check of cost function compared against cost calculated during trusted run of SLQ
 */
TEST(HybridSlqTest, state_rollout_slq) {
  using namespace ocs2;

  constexpr scalar_t minRelCost = 1e-6;  // to avoid early termination

  const ddp::Settings ddpSettings = [&]() {
    ddp::Settings settings;

    settings.algorithm_ = ddp::Algorithm::SLQ;
    settings.displayInfo_ = true;
    settings.displayShortSummary_ = true;
    settings.maxNumIterations_ = 100;
    settings.nThreads_ = 1;
    settings.minRelCost_ = minRelCost;
    settings.checkNumericalStability_ = false;
    settings.absTolODE_ = 1e-10;
    settings.relTolODE_ = 1e-7;
    settings.maxNumStepsPerSecond_ = 10000;
    settings.useFeedbackPolicy_ = true;
    settings.debugPrintRollout_ = false;
    settings.strategy_ = search_strategy::Type::LINE_SEARCH;
    settings.lineSearch_.minStepLength = 1e-4;

    return settings;
  }();

  const rollout::Settings rolloutSettings = [&]() {
    rollout::Settings settings;
    settings.absTolODE = 1e-10;
    settings.relTolODE = 1e-7;
    settings.timeStep = 1e-3;
    settings.maxNumStepsPerSecond = 10000;
    return settings;
  }();

  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 5.0;
  const vector_t initState = (vector_t(STATE_DIM) << 0.0, 1.0, 1.0).finished();

  // rollout
  HybridSysDynamics systemDynamics;
  StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // cost function
  const matrix_t Q = (matrix_t(STATE_DIM, STATE_DIM) << 50, 0, 0, 0, 50, 0, 0, 0, 0).finished();
  const matrix_t R = (matrix_t(INPUT_DIM, INPUT_DIM) << 1).finished();
  // constraints
  auto boundsConstraints = std::make_unique<HybridSysBounds>();

  OptimalControlProblem problem;
  problem.dynamicsPtr.reset(systemDynamics.clone());
  problem.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
  problem.preJumpCostPtr->add("preJumpCost", std::make_unique<QuadraticStateCost>(Q));
  problem.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Q));
  problem.inequalityLagrangianPtr->add("bounds",
                                       create(std::move(boundsConstraints), augmented::SlacknessSquaredHingePenalty::create({200.0, 0.1})));

  const vector_t xNominal = vector_t::Zero(STATE_DIM);
  const vector_t uNominal = vector_t::Zero(INPUT_DIM);
  TargetTrajectories targetTrajectories({startTime}, {xNominal}, {uNominal});
  auto referenceManager = std::make_shared<ReferenceManager>(std::move(targetTrajectories));

  // operatingTrajectories
  const vector_t stateOperatingPoint = vector_t::Zero(STATE_DIM);
  const vector_t inputOperatingPoint = vector_t::Zero(INPUT_DIM);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  // Test 1: Check constraint compliance. It uses a solver observer to get metrics for the bounds constraints
  auto boundsConstraintsObserverPtr = SolverObserver::LagrangianTermObserver(
      SolverObserver::Type::Intermediate, "bounds",
      [&](const scalar_array_t& timeTraj, const std::vector<LagrangianMetricsConstRef>& metricsTraj) {
        constexpr scalar_t constraintViolationTolerance = 1e-1;
        for (size_t i = 0; i < metricsTraj.size(); i++) {
          const vector_t constraintViolation = metricsTraj[i].constraint.cwiseMin(0.0);
          EXPECT_NEAR(constraintViolation(0), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(1), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(2), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
          EXPECT_NEAR(constraintViolation(3), 0.0, constraintViolationTolerance) << "At time " << timeTraj[i] << "\n";
        }
      });

  // setup SLQ
  SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.setReferenceManager(referenceManager);
  slq.addSolverObserver(std::move(boundsConstraintsObserverPtr));

  // run SLQ
  slq.run(startTime, initState, finalTime);
  const auto solution = slq.primalSolution(finalTime);

  // Test 2: No penetration of guard surfaces
  for (int i = 0; i < solution.timeTrajectory_.size(); i++) {
    const vector_t guardSurfaces = systemDynamics.computeGuardSurfaces(solution.timeTrajectory_[i], solution.stateTrajectory_[i]);
    EXPECT_GT(guardSurfaces(0), -1e-10) << solution.timeTrajectory_[i] << ": " << guardSurfaces(0) << ", " << guardSurfaces(1) << "\n";
    EXPECT_GT(guardSurfaces(1), -1e-10) << solution.timeTrajectory_[i] << ": " << guardSurfaces(0) << ", " << guardSurfaces(1) << "\n";
  }

  // Test 3: Check of cost
  const auto performanceIndecesST = slq.getPerformanceIndeces();
  EXPECT_LT(performanceIndecesST.cost - 20.1, 10.0 * minRelCost);
}
