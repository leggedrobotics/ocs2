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
#include <ocs2_oc/test/dynamics_hybrid_slq_test.h>

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

  const size_t stateDim = STATE_DIM;
  const size_t inputDim = INPUT_DIM;

  ddp::Settings ddpSettings;
  ddpSettings.algorithm_ = ddp::Algorithm::SLQ;
  ddpSettings.displayInfo_ = true;
  ddpSettings.displayShortSummary_ = true;
  ddpSettings.maxNumIterations_ = 30;
  ddpSettings.nThreads_ = 1;
  ddpSettings.checkNumericalStability_ = false;
  ddpSettings.absTolODE_ = 1e-10;
  ddpSettings.relTolODE_ = 1e-7;
  ddpSettings.maxNumStepsPerSecond_ = 10000;
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.debugPrintRollout_ = false;
  ddpSettings.strategy_ = search_strategy::Type::LINE_SEARCH;
  ddpSettings.lineSearch_.minStepLength_ = 0.001;

  rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-10;
  rolloutSettings.relTolODE = 1e-7;
  rolloutSettings.timeStep = 1e-3;
  rolloutSettings.maxNumStepsPerSecond = 10000;

  scalar_t startTime = 0.0;
  scalar_t finalTime = 5.0;

  vector_t initState(stateDim);
  initState << 0, 1, 1;

  // rollout
  HybridSysDynamics systemDynamics;
  StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // constraints
  std::unique_ptr<StateInputConstraint> systemConstraints(new HybridSysBounds);
  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty({0.2, 1e-4}));
  std::unique_ptr<StateInputCost> softSystemLagrangian(new StateInputSoftConstraint(std::move(systemConstraints), std::move(penalty)));

  // cost function
  matrix_t Q(stateDim, stateDim);
  Q << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  matrix_t R(inputDim, inputDim);
  R << 1;
  std::unique_ptr<ocs2::StateInputCost> cost(new QuadraticStateInputCost(Q, R));
  matrix_t Qf(stateDim, stateDim);
  Qf << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  std::unique_ptr<ocs2::StateCost> preJumpCost(new QuadraticStateCost(Qf));
  std::unique_ptr<ocs2::StateCost> finalCost(new QuadraticStateCost(Qf));

  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr.reset(systemDynamics.clone());
  problem.costPtr->add("cost", std::move(cost));
  problem.preJumpCostPtr->add("preJumpCost", std::move(preJumpCost));
  problem.finalCostPtr->add("finalCost", std::move(finalCost));
  problem.softConstraintPtr->add("bounds", std::move(softSystemLagrangian));

  vector_t xNominal = vector_t::Zero(stateDim);
  vector_t uNominal = vector_t::Zero(inputDim);
  TargetTrajectories targetTrajectories({startTime}, {xNominal}, {uNominal});
  auto referenceManager = std::make_shared<ReferenceManager>(std::move(targetTrajectories));

  // operatingTrajectories
  vector_t stateOperatingPoint = vector_t::Zero(stateDim);
  vector_t inputOperatingPoint = vector_t::Zero(inputDim);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  std::cout << "Starting SLQ Procedure" << std::endl;

  // SLQ
  SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.setReferenceManager(referenceManager);
  slq.run(startTime, initState, finalTime);
  auto solution = slq.primalSolution(finalTime);
  std::cout << "SLQ Procedure Done" << std::endl;

  if (false) {
    for (int i = 0; i < solution.stateTrajectory_.size(); i++) {
      std::cout << i << ";" << solution.timeTrajectory_[i] << ";" << solution.stateTrajectory_[i][0] << ";"
                << solution.stateTrajectory_[i][1] << ";" << solution.stateTrajectory_[i][2] << ";" << solution.inputTrajectory_[i]
                << std::endl;
    }
  }

  for (int i = 0; i < solution.stateTrajectory_.size(); i++) {
    // Test 1 : Constraint Compliance
    scalar_t constraint0 = -solution.inputTrajectory_[i][0] + 2;
    scalar_t constraint1 = solution.inputTrajectory_[i][0] + 2;
    scalar_t constraint2 = solution.stateTrajectory_[i][0] + 2;
    scalar_t constraint3 = -solution.stateTrajectory_[i][0] + 2;

    EXPECT_GT(constraint0, 0);
    EXPECT_GT(constraint1, 0);
    EXPECT_GT(constraint2, 0);
    EXPECT_GT(constraint3, 0);

    // Test 2 : No penetration of guardSurfaces
    vector_t guardSurfaces = systemDynamics.computeGuardSurfaces(solution.timeTrajectory_[i], solution.stateTrajectory_[i]);

    EXPECT_GT(guardSurfaces[0], -1e-10);
    if (!(guardSurfaces[0] > -1e-10)) {
      std::cout << solution.timeTrajectory_[i] << "," << guardSurfaces[0] << "," << guardSurfaces[1] << std::endl;
    }

    EXPECT_GT(guardSurfaces[1], -1e-10);
    if (!(guardSurfaces[1] > -1e-10)) {
      std::cout << solution.timeTrajectory_[i] << "," << guardSurfaces[0] << "," << guardSurfaces[1] << std::endl;
    }
  }

  // Test 3: Check of cost function
  auto performanceIndecesST = slq.getPerformanceIndeces();
  EXPECT_LT(performanceIndecesST.cost - 13.0, 10.0 * ddpSettings.minRelCost_);
}
