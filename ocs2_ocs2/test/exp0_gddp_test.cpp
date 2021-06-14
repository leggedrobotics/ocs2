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
#include <ocs2_oc/test/EXP0.h>

#include <ocs2_ddp/SLQ.h>

#include <ocs2_ocs2/GDDP.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 1 };

TEST(exp0_gddp_test, optimum_gradient_test) {
  SLQ_Settings slqSettings;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slqSettings.ddpSettings_.displayShortSummary_ = false;
  slqSettings.ddpSettings_.preComputeRiccatiTerms_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.nThreads_ = 1;  // single threaded
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.minRelCost_ = 5e-4;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.strategy_ = ddp_strategy::type::LINE_SEARCH;
  slqSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.0001;

  rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-10;
  rolloutSettings.relTolODE = 1e-7;
  rolloutSettings.maxNumStepsPerSecond = 10000;

  GDDP_Settings gddpSettings;
  gddpSettings.displayInfo_ = true;
  gddpSettings.checkNumericalStability_ = false;
  gddpSettings.nThreads_ = 3;
  gddpSettings.useLQForDerivatives_ = false;
  gddpSettings.absTolODE_ = 1e-10;
  gddpSettings.relTolODE_ = 1e-7;
  gddpSettings.maxNumStepsPerSecond_ = 10000;

  // event times
  std::vector<double> optimumEventTimes{0.1897};
  std::vector<size_t> subsystemsSequence{0, 1};
  std::shared_ptr<ModeScheduleManager<STATE_DIM, INPUT_DIM>> modeScheduleManagerPtr(
      new ModeScheduleManager<STATE_DIM, INPUT_DIM>({optimumEventTimes, subsystemsSequence}));

  double startTime = 0.0;
  double finalTime = 2.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(1.0);
  partitioningTimes.push_back(finalTime);

  Eigen::Vector2d initState(0.0, 2.0);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system dynamics
  EXP0_System systemDynamics(modeScheduleManagerPtr);
  TimeTriggeredRollout<STATE_DIM, INPUT_DIM> timeTriggeredRollout(systemDynamics, rolloutSettings);

  // system derivatives
  EXP0_SystemDerivative systemDerivative(modeScheduleManagerPtr);

  // system constraints
  ConstraintBase systemConstraint;

  // system cost functions
  EXP0_CostFunction systemCostFunction(modeScheduleManagerPtr);

  // system operatingTrajectories
  Eigen::Matrix<double, STATE_DIM, 1> stateOperatingPoint = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
  Eigen::Matrix<double, INPUT_DIM, 1> inputOperatingPoint = Eigen::Matrix<double, INPUT_DIM, 1>::Zero();
  EXP0_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single core version
  SLQ<STATE_DIM, INPUT_DIM> slqST(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
                                  slqSettings);
  slqST.setModeScheduleManager(modeScheduleManagerPtr);

  // SLQ data collector
  SLQ_DataCollector<STATE_DIM, INPUT_DIM> slqDataCollector(&timeTriggeredRollout, &systemDerivative, &systemConstraint,
                                                           &systemCostFunction);
  // GDDP
  GDDP<STATE_DIM, INPUT_DIM> gddp(gddpSettings);

  // run SLQ
  slqST.run(startTime, initState, finalTime, partitioningTimes);
  slqDataCollector.collect(&slqST);

  // cost
  const auto performanceIndex = slqST.getPerformanceIndeces();

  // run GDDP using LQ
  gddp.settings().useLQForDerivatives_ = true;
  gddp.run(&slqDataCollector);
  // cost derivative
  Eigen::Matrix<double, 1, 1> costFunctionDerivative_LQ;
  gddp.getCostFuntionDerivative(costFunctionDerivative_LQ);

  // run GDDP using BVP
  gddp.settings().useLQForDerivatives_ = false;
  gddp.run(&slqDataCollector);
  // cost derivative
  Eigen::Matrix<double, 1, 1> costFunctionDerivative_BVP;
  gddp.getCostFuntionDerivative(costFunctionDerivative_BVP);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::cerr << "### Optimum cost is: " << performanceIndex.totalCost << "\n";
  std::cerr << "### Optimum event times are:            ["
            << Eigen::Map<Eigen::VectorXd>(optimumEventTimes.data(), optimumEventTimes.size()).transpose() << "]\n";
  std::cerr << "### Optimum cost derivative LQ method:  [" << costFunctionDerivative_LQ.transpose() << "]\n";
  std::cerr << "### Optimum cost derivative BVP method: [" << costFunctionDerivative_BVP.transpose() << "]\n";

  ASSERT_LT(costFunctionDerivative_LQ.norm() / fabs(performanceIndex.totalCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: GDDP failed in the EXP0's cost derivative LQ test!";
  ASSERT_LT(costFunctionDerivative_BVP.norm() / fabs(performanceIndex.totalCost), 10 * slqSettings.ddpSettings_.minRelCost_)
      << "MESSAGE: GDDP failed in the EXP0's cost derivative BVP test!";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
