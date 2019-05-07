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

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <gtest/gtest.h>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>

#include <ocs2_oc/test/EXP0.h>

using namespace ocs2;

enum {
  STATE_DIM = 2,
  INPUT_DIM = 1
};

TEST(exp0_slq_test, exp0_slq_test) {
  // system dynamics
  EXP0_System systemDynamics;

  // system derivatives
  EXP0_SystemDerivative systemDerivative;

  // system constraints
  EXP0_SystemConstraint systemConstraint;

  // system cost functions
  EXP0_CostFunction systemCostFunction;

  // system operatingTrajectories
  Eigen::Matrix<double, 2, 1> stateOperatingPoint = Eigen::Matrix<double, 2, 1>::Zero();
  Eigen::Matrix<double, 1, 1> inputOperatingPoint = Eigen::Matrix<double, 1, 1>::Zero();
  EXP0_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);


  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  SLQ_Settings slqSettings;
  slqSettings.displayInfo_ = false;
  slqSettings.displayShortSummary_ = true;
  slqSettings.absTolODE_ = 1e-10;
  slqSettings.relTolODE_ = 1e-7;
  slqSettings.maxNumStepsPerSecond_ = 10000;
  slqSettings.nThreads_ = 3;
  slqSettings.maxNumIterationsSLQ_ = 30;
  slqSettings.lsStepsizeGreedy_ = true;
  slqSettings.noStateConstraints_ = true;
  slqSettings.minLearningRateSLQ_ = 0.0001;
  slqSettings.minRelCostSLQ_ = 5e-4;
  slqSettings.checkNumericalStability_ = false;

  slqSettings.rolloutSettings_.absTolODE_ = 1e-10;
  slqSettings.rolloutSettings_.relTolODE_ = 1e-7;
  slqSettings.rolloutSettings_.maxNumStepsPerSecond_ = 10000;

  // switching times
  std::vector<double> switchingTimes{0.1897};
  EXP0_LogicRules logicRules(switchingTimes);

  double startTime = 0.0;
  double finalTime = 2.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(switchingTimes[0]);
  partitioningTimes.push_back(finalTime);

  Eigen::Vector2d initState(0.0, 2.0);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single core version
  SLQ<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq(
      &systemDynamics, &systemDerivative,
      &systemConstraint, &systemCostFunction,
      &operatingTrajectories, slqSettings, &logicRules);

  // SLQ - multi-core version
//  SLQ_MP<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq_mp(
//		  &systemDynamics, &systemDerivative,
//		  &systemConstraint, &systemCostFunction,
//		  &operatingTrajectories, slqSettings, &logicRules);

  // run single core SLQ
  if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  slq.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-core SLQ
//  if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
//	  std::cerr << "\n>>> multi-core SLQ" << std::endl;
//  slq_mp.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get controller
  SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock = slq.getController();
//  SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStockMT = slq_mp.getController();

  // get performance indices
  double totalCost, totalCost_mp;
  double constraint1ISE, constraint1ISE_mp;
  double constraint2ISE, constraint2ISE_mp;
  slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
//  slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const double expectedCost = 9.7667;
  ASSERT_LT(fabs(totalCost - expectedCost), 10 * slqSettings.minRelCostSLQ_) <<
		  "MESSAGE: SLQ failed in the EXP0's cost test!";
//  ASSERT_LT(fabs(totalCost_mp - expectedCost), 10*slqSettings.minRelCostSLQ_) <<
//		  "MESSAGE: SLQ_MP failed in the EXP1's cost test!";

  const double expectedISE1 = 0.0;
  ASSERT_LT(fabs(constraint1ISE - expectedISE1), 10 * slqSettings.minRelConstraint1ISE_) <<
		  "MESSAGE: SLQ failed in the EXP0's type-1 constraint ISE test!";
//  ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10*slqSettings.minRelConstraint1ISE_) <<
//		  "MESSAGE: SLQ_MP failed in the EXP1's type-1 constraint ISE test!";

  const double expectedISE2 = 0.0;
  ASSERT_LT(fabs(constraint2ISE - expectedISE2), 10 * slqSettings.minRelConstraint1ISE_) <<
		  "MESSAGE: SLQ failed in the EXP0's type-2 constraint ISE test!";
//  ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10*slqSettings.minRelConstraint1ISE_) <<
//		  "MESSAGE: SLQ_MP failed in the EXP1's type-2 constraint ISE test!";
}

// This test hangs on the buildserver in an infinite loop. Issue likely multithreading related.
// Locally it runs fine for several users.
//TEST(exp0_slq_test, DISABLED_exp0_slq_test_multi_core) {
//
//  // system dynamics
//  EXP0_System systemDynamics;
//
//  // system derivatives
//  EXP0_SystemDerivative systemDerivative;
//
//  // system constraints
//  EXP0_SystemConstraint systemConstraint;
//
//  // system cost functions
//  EXP0_CostFunction systemCostFunction;
//
//  // system operatingTrajectories
//  Eigen::Matrix<double, 2, 1> stateOperatingPoint = Eigen::Matrix<double, 2, 1>::Zero();
//  Eigen::Matrix<double, 1, 1> inputOperatingPoint = Eigen::Matrix<double, 1, 1>::Zero();
//  EXP0_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);
//
//
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  SLQ_Settings slqSettings;
//  slqSettings.displayInfo_ = true;
//  slqSettings.displayShortSummary_ = true;
//  slqSettings.absTolODE_ = 1e-10;
//  slqSettings.relTolODE_ = 1e-7;
//  slqSettings.maxNumStepsPerSecond_ = 10000;
//  slqSettings.nThreads_ = 3;
//  slqSettings.maxNumIterationsSLQ_ = 30;
//  slqSettings.lsStepsizeGreedy_ = true;
//  slqSettings.noStateConstraints_ = true;
//  slqSettings.minLearningRateSLQ_ = 0.0001;
//  slqSettings.minRelCostSLQ_ = 5e-4;
//  slqSettings.checkNumericalStability_ = false;
//
//  // switching times
//  std::vector<double> switchingTimes{0.1897};
//  EXP0_LogicRules logicRules(switchingTimes);
//
//  double startTime = 0.0;
//  double finalTime = 2.0;
//
//  // partitioning times
//  std::vector<double> partitioningTimes;
//  partitioningTimes.push_back(startTime);
//  partitioningTimes.push_back(switchingTimes[0]);
//  partitioningTimes.push_back(finalTime);
//
//  Eigen::Vector2d initState(0.0, 2.0);
//
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//
//  // GSLQ MP version
//  SLQ_MP<STATE_DIM, INPUT_DIM, EXP0_LogicRules> slq_mp(
//      &systemDynamics, &systemDerivative,
//      &systemConstraint, &systemCostFunction,
//      &operatingTrajectories, slqSettings, &logicRules);
//
//  // run multi-core SLQ
//  if (slqSettings.displayInfo_ || slqSettings.displayShortSummary_)
//    std::cerr << "\n>>> multi-core SLQ" << std::endl;
//  slq_mp.run(startTime, initState, finalTime, partitioningTimes);
//
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  // get controller
//  SLQ_BASE<STATE_DIM, INPUT_DIM, EXP0_LogicRules>::controller_array_t controllersStock_mp = slq_mp.getController();
//
//  // get performance indices
//  double totalCost, totalCost_mp;
//  double constraint1ISE_mp;
//  double constraint2ISE_mp;
//  slq_mp.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);
//
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  /******************************************************************************************************/
//  const double expectedCost = 9.7667;
//  ASSERT_LT(fabs(totalCost_mp - expectedCost), 10 * slqSettings.minRelCostSLQ_) <<
//                                                                                  "MESSAGE: SLQ_MP failed in the EXP0's cost test!";
//
//  const double expectedISE1 = 0.0;
//  ASSERT_LT(fabs(constraint1ISE_mp - expectedISE1), 10 * slqSettings.minRelConstraint1ISE_) <<
//                                                                                            "MESSAGE: SLQ_MP failed in the EXP0's type-1 constraint ISE test!";
//
//  const double expectedISE2 = 0.0;
//  ASSERT_LT(fabs(constraint2ISE_mp - expectedISE2), 10 * slqSettings.minRelConstraint1ISE_) <<
//                                                                                            "MESSAGE: SLQ_MP failed in the EXP0's type-2 constraint ISE test!";
//}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
