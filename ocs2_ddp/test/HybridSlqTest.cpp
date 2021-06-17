#include <gtest/gtest.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/test/dynamics_hybrid_slq_test.h>

#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>

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
  ddpSettings.inequalityConstraintMu_ = 0.1;
  ddpSettings.inequalityConstraintDelta_ = 1e-4;
  ddpSettings.checkNumericalStability_ = false;
  ddpSettings.absTolODE_ = 1e-10;
  ddpSettings.relTolODE_ = 1e-7;
  ddpSettings.maxNumStepsPerSecond_ = 10000;
  ddpSettings.useNominalTimeForBackwardPass_ = true;
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.debugPrintRollout_ = false;
  ddpSettings.strategy_ = search_strategy::Type::LINE_SEARCH;

  rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-10;
  rolloutSettings.relTolODE = 1e-7;
  rolloutSettings.timeStep = 1e-3;
  rolloutSettings.maxNumStepsPerSecond = 10000;

  scalar_t startTime = 0.0;
  scalar_t finalTime = 5.0;

  std::vector<scalar_t> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime);

  vector_t initState(stateDim);
  initState << 0, 1, 1;

  // rollout
  hybridSysDynamics systemDynamics;
  StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // constraints
  hybridSysConstraints systemConstraints;
  // cost function
  matrix_t Q(stateDim, stateDim);
  Q << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  matrix_t Qf(stateDim, stateDim);
  Qf << 50, 0, 0, 0, 50, 0, 0, 0, 0;
  matrix_t R(inputDim, inputDim);
  R << 1;
  vector_t xNominal = vector_t::Zero(stateDim);
  vector_t uNominal = vector_t::Zero(inputDim);
  TargetTrajectories targetTrajectories({startTime}, {xNominal}, {uNominal});

  QuadraticCostFunction systemCost(Q, R, Qf);

  // operatingTrajectories
  vector_t stateOperatingPoint = vector_t::Zero(stateDim);
  vector_t inputOperatingPoint = vector_t::Zero(inputDim);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  std::cout << "Starting SLQ Procedure" << std::endl;
  // SLQ
  SLQ slq(&stateTriggeredRollout, &systemDynamics, &systemConstraints, &systemCost, &operatingTrajectories, ddpSettings);
  slq.setTargetTrajectories(targetTrajectories);
  slq.run(startTime, initState, finalTime, partitioningTimes);
  auto solution = slq.primalSolution(finalTime);
  std::cout << "SLQ Procedure Done" << std::endl;

  if (false) {
    for (int i = 0; i < solution.stateTrajectory_.size(); i++) {
      std::cout << i << ";" << solution.timeTrajectory_[i] << ";" << solution.stateTrajectory_[i][0] << ";"
                << solution.stateTrajectory_[i][1] << ";" << solution.stateTrajectory_[i][2] << ";" << solution.inputTrajectory_[i]
                << std::endl;
    }
  }

  scalar_t cost;
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
  EXPECT_LT(std::fabs(performanceIndecesST.totalCost - 20.08), 10.0 * ddpSettings.minRelCost_);
}
