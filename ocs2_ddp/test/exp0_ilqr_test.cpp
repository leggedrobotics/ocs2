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

#include <ocs2_ddp/ILQR.h>

#include <ocs2_oc/test/EXP0.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(exp0_ilqr_test, exp0_ilqr_test) {
  using linear_controller_t = ILQR<STATE_DIM, INPUT_DIM>::linear_controller_t;
  using feedforward_controller_t = ILQR<STATE_DIM, INPUT_DIM>::feedforward_controller_t;

  ILQR_Settings ilqrSettings;
  ilqrSettings.ddpSettings_.displayInfo_ = true;
  ilqrSettings.ddpSettings_.displayShortSummary_ = true;
  ilqrSettings.ddpSettings_.absTolODE_ = 1e-10;
  ilqrSettings.ddpSettings_.relTolODE_ = 1e-7;
  ilqrSettings.ddpSettings_.maxNumStepsPerSecond_ = 1000000;
  ilqrSettings.ddpSettings_.maxNumIterations_ = 30;
  ilqrSettings.ddpSettings_.minLearningRate_ = 0.0001;
  ilqrSettings.ddpSettings_.minRelCost_ = 5e-4;
  ilqrSettings.ddpSettings_.checkNumericalStability_ = false;
  ilqrSettings.ddpSettings_.useFeedbackPolicy_ = true;
  ilqrSettings.ddpSettings_.debugPrintRollout_ = false;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  // event times
  std::vector<double> eventTimes{0.1897};
  std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr(
      new ModeScheduleManager<STATE_DIM, INPUT_DIM>({eventTimes, subsystemsSequence}));

  double startTime = 0.0;
  double finalTime = 2.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(eventTimes[0]);
  partitioningTimes.push_back(finalTime);

  EXP0_System::state_vector_t initState(0.0, 2.0);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP0_System systemDynamics(modeScheduleManagerPtr);
  TimeTriggeredRollout<STATE_DIM, INPUT_DIM> timeTriggeredRollout(systemDynamics, rolloutSettings);

  // system derivatives
  EXP0_SystemDerivative systemDerivative(modeScheduleManagerPtr);

  // system constraints
  EXP0_SystemConstraint systemConstraint;

  // system cost functions
  EXP0_CostFunction systemCostFunction(modeScheduleManagerPtr);

  // system operatingTrajectories
  Eigen::Matrix<double, 2, 1> stateOperatingPoint = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 1, 1> inputOperatingPoint = Eigen::Matrix<double, 1, 1>::Zero();
  EXP0_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // ILQR - single-threaded version
  ilqrSettings.ddpSettings_.nThreads_ = 1;
  ILQR<STATE_DIM, INPUT_DIM> ilqrST(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction,
                                    &operatingTrajectories, ilqrSettings);
  ilqrST.setModeScheduleManager(modeScheduleManagerPtr);

  // ILQR - multi-threaded version
  ilqrSettings.ddpSettings_.nThreads_ = 3;
  ILQR<STATE_DIM, INPUT_DIM> ilqrMT(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction,
                                    &operatingTrajectories, ilqrSettings);
  ilqrMT.setModeScheduleManager(modeScheduleManagerPtr);

  // run single_threaded core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-threaded ILQR" << std::endl;
  }
  ilqrST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-threaded ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> multi-threaded ILQR" << std::endl;
  }
  ilqrMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  ILQR<STATE_DIM, INPUT_DIM>::primal_solution_t solutionST = ilqrST.primalSolution(finalTime);
  ILQR<STATE_DIM, INPUT_DIM>::primal_solution_t solutionMT = ilqrMT.primalSolution(finalTime);

  // get performance indices
  auto performanceIndecesST = ilqrST.getPerformanceIndeces();
  auto performanceIndecesMT = ilqrMT.getPerformanceIndeces();

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const double expectedCost = 9.7667;
  ASSERT_LT(fabs(performanceIndecesST.totalCost - expectedCost), 10 * ilqrSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: ILQR_ST failed in the EXP0's cost test!";
  ASSERT_LT(fabs(performanceIndecesMT.totalCost - expectedCost), 10 * ilqrSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: ILQR_MT failed in the EXP1's cost test!";

  const double expectedISE1 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateInputEqConstraintISE - expectedISE1), 10 * ilqrSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: ILQR_ST failed in the EXP0's type-1 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateInputEqConstraintISE - expectedISE1), 10 * ilqrSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: ILQR_MT failed in the EXP1's type-1 constraint ISE test!";

  const double expectedISE2 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateEqConstraintISE - expectedISE2), 10 * ilqrSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: ILQR_ST failed in the EXP0's type-2 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateEqConstraintISE - expectedISE2), 10 * ilqrSettings.ddpSettings_.minRelConstraint1ISE_)
      << "MESSAGE: ILQR_MT failed in the EXP1's type-2 constraint ISE test!";

  double ctrlFinalTime;
  if (ilqrSettings.ddpSettings_.useFeedbackPolicy_) {
    ctrlFinalTime = dynamic_cast<linear_controller_t*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  } else {
    ctrlFinalTime = dynamic_cast<feedforward_controller_t*>(solutionST.controllerPtr_.get())->timeStamp_.back();
  }
  ASSERT_DOUBLE_EQ(solutionST.timeTrajectory_.back(), finalTime) << "MESSAGE: ILQR_ST failed in policy final time of trajectory!";
  ASSERT_DOUBLE_EQ(ctrlFinalTime, finalTime) << "MESSAGE: ILQR_ST failed in policy final time of controller!";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
