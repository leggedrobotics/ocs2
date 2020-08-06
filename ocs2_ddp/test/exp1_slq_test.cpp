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

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/test/EXP1.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(exp1_slq_test, exp1_slq_test) {
  ddp::Settings ddpSettings;
  ddpSettings.algorithm_ = ddp::algorithm::SLQ;
  ddpSettings.preComputeRiccatiTerms_ = false;
  ddpSettings.displayInfo_ = true;
  ddpSettings.displayShortSummary_ = true;
  ddpSettings.maxNumIterations_ = 30;
  ddpSettings.maxNumIterations_ = 30;
  ddpSettings.checkNumericalStability_ = true;
  ddpSettings.absTolODE_ = 1e-10;
  ddpSettings.relTolODE_ = 1e-7;
  ddpSettings.maxNumStepsPerSecond_ = 10000;
  ddpSettings.useNominalTimeForBackwardPass_ = true;
  ddpSettings.useFeedbackPolicy_ = false;
  ddpSettings.debugPrintRollout_ = false;
  ddpSettings.strategy_ = ddp_strategy::type::LINE_SEARCH;
  ddpSettings.lineSearch_.minStepLength_ = 0.0001;

  rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // event times
  std::vector<scalar_t> eventTimes{0.2262, 1.0176};
  std::vector<size_t> subsystemsSequence{0, 1, 2};
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr(new ModeScheduleManager({eventTimes, subsystemsSequence}));

  scalar_t startTime = 0.0;
  scalar_t finalTime = 3.0;

  // partitioning times
  std::vector<scalar_t> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(eventTimes[0]);
  partitioningTimes.push_back(eventTimes[1]);
  partitioningTimes.push_back(finalTime);

  vector_t initState(2);
  initState << 2.0, 3.0;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP1_System systemDynamics(modeScheduleManagerPtr);
  TimeTriggeredRollout timeTriggeredRollout(systemDynamics, rolloutSettings);

  // system constraints
  ConstraintBase systemConstraint;

  // system cost functions
  EXP1_CostFunction systemCostFunction(modeScheduleManagerPtr);

  // system operatingTrajectories
  vector_t stateOperatingPoint = vector_t::Zero(STATE_DIM);
  vector_t inputOperatingPoint = vector_t::Zero(INPUT_DIM);
  OperatingPoints operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single core version
  ddpSettings.nThreads_ = 1;
  SLQ slqST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ddpSettings);
  slqST.setModeScheduleManager(modeScheduleManagerPtr);

  ddpSettings.nThreads_ = 3;
  // SLQ - multi-threaded version
  SLQ slqMT(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ddpSettings);
  slqMT.setModeScheduleManager(modeScheduleManagerPtr);

  // run single-threaded SLQ
  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  }
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-threaded SLQ
  if (ddpSettings.displayInfo_ || ddpSettings.displayShortSummary_) {
    std::cerr << "\n>>> multi-core SLQ" << std::endl;
  }
  slqMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  auto solutionST = slqST.primalSolution(finalTime);
  auto solutionMT = slqMT.primalSolution(finalTime);

  // get performance indices
  auto performanceIndecesST = slqST.getPerformanceIndeces();
  auto performanceIndecesMT = slqMT.getPerformanceIndeces();

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const scalar_t expectedCost = 5.4399;
  ASSERT_LT(fabs(performanceIndecesST.totalCost - expectedCost), 10 * ddpSettings.minRelCost_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's cost test!";
  ASSERT_LT(fabs(performanceIndecesMT.totalCost - expectedCost), 10 * ddpSettings.minRelCost_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's cost test!";

  const scalar_t expectedISE1 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateInputEqConstraintISE - expectedISE1), 10 * ddpSettings.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateInputEqConstraintISE - expectedISE1), 10 * ddpSettings.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";

  const scalar_t expectedISE2 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateEqConstraintISE - expectedISE2), 10 * ddpSettings.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateEqConstraintISE - expectedISE2), 10 * ddpSettings.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";

  scalar_t ctrlFinalTime;
  if (ddpSettings.useFeedbackPolicy_) {
    ctrlFinalTime = dynamic_cast<LinearController*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  } else {
    ctrlFinalTime = dynamic_cast<FeedforwardController*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  }
  ASSERT_DOUBLE_EQ(solutionST.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ_ST failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: SLQ_ST failed in policy final time of controller!";
}
