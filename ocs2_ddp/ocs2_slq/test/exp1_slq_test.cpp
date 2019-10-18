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

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>

#include <ocs2_oc/test/EXP1.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(exp1_slq_test, Exp1_slq_test) {
  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.nThreads_ = 3;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.lsStepsizeGreedy_ = true;
  slqSettings.ddpSettings_.noStateConstraints_ = true;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = false;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // switching times
  std::vector<double> eventTimes{0.2262, 1.0176};
  std::vector<size_t> subsystemsSequence{0, 1, 2};
  std::shared_ptr<EXP1_LogicRules> logicRules(new EXP1_LogicRules(eventTimes, subsystemsSequence));

  double startTime = 0.0;
  double finalTime = 3.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(eventTimes[0]);
  partitioningTimes.push_back(eventTimes[1]);
  partitioningTimes.push_back(finalTime);

  EXP1_System::state_vector_t initState(2.0, 3.0);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP1_System systemDynamics(logicRules);
  TimeTriggeredRollout<STATE_DIM, INPUT_DIM> timeTriggeredRollout(systemDynamics, rolloutSettings);

  // system derivatives
  EXP1_SystemDerivative systemDerivative(logicRules);

  // system constraints
  EXP1_SystemConstraint systemConstraint;

  // system cost functions
  EXP1_CostFunction systemCostFunction(logicRules);

  // system operatingTrajectories
  Eigen::Matrix<double, STATE_DIM, 1> stateOperatingPoint = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
  Eigen::Matrix<double, INPUT_DIM, 1> inputOperatingPoint = Eigen::Matrix<double, INPUT_DIM, 1>::Zero();
  EXP1_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single core version
  SLQ<STATE_DIM, INPUT_DIM> slqST(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
                                  slqSettings, logicRules);

  // SLQ - multi-threaded version
  SLQ_MP<STATE_DIM, INPUT_DIM> slqMT(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
                                     slqSettings, logicRules);

  // run single-threaded SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_)
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-threaded SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_)
    std::cerr << "\n>>> multi-core SLQ" << std::endl;
  slqMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  SLQ_BASE<STATE_DIM, INPUT_DIM>::primal_solution_t solutionST = slqST.primalSolution(finalTime);
  SLQ_BASE<STATE_DIM, INPUT_DIM>::primal_solution_t solutionMT = slqMT.primalSolution(finalTime);

  // get performance indices
  double totalCostST, totalCostMT;
  double constraint1ISE_ST, constraint1ISE_MT;
  double constraint2ISE_ST, constraint2ISE_MT;
  slqST.getPerformanceIndeces(totalCostST, constraint1ISE_ST, constraint2ISE_ST);
  slqMT.getPerformanceIndeces(totalCostMT, constraint1ISE_MT, constraint2ISE_MT);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const double expectedCost = 5.4399;
  ASSERT_LT(fabs(totalCostST - expectedCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's cost test!";
  ASSERT_LT(fabs(totalCostMT - expectedCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's cost test!";

  const double expectedISE1 = 0.0;
  ASSERT_LT(fabs(constraint1ISE_ST - expectedISE1), 10 * slqSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";
  ASSERT_LT(fabs(constraint1ISE_MT - expectedISE1), 10 * slqSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-1 constraint ISE test!";

  const double expectedISE2 = 0.0;
  ASSERT_LT(fabs(constraint2ISE_ST - expectedISE2), 10 * slqSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: single-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";
  ASSERT_LT(fabs(constraint2ISE_MT - expectedISE2), 10 * slqSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: multi-threaded SLQ failed in the EXP1's type-2 constraint ISE test!";

  double ctrlFinalTime;
  if (slqSettings.ddpSettings_.useFeedbackPolicy_) {
    ctrlFinalTime = dynamic_cast<SLQ<STATE_DIM, INPUT_DIM>::linear_controller_t*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  } else {
    ctrlFinalTime = dynamic_cast<SLQ<STATE_DIM, INPUT_DIM>::feedforward_controller_t*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  }
  ASSERT_DOUBLE_EQ(solutionST.timeTrajectory_.back(), finalTime) << "MESSAGE: SLQ_ST failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: SLQ_ST failed in policy final time of controller!";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
