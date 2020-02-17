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

#include <ocs2_ddp/SLQ.h>

#include <ocs2_oc/test/EXP2.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 2 };

// This test have problem. It does not converge properly.
TEST(exp2_slq_test, DISABLED_exp2_slq_test) {
  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = false;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.maxNumIterations_ = 200;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.constraintStepSize_ = 0.1;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.nThreads_ = 1;  // single threaded
  slqSettings.ddpSettings_.useMakePSD_ = true;
  slqSettings.ddpSettings_.noStateConstraints_ = true;
  slqSettings.ddpSettings_.checkNumericalStability_ = true;

  // event times
  std::vector<double> eventTimes{0.2, 1.2};
  std::vector<size_t> subsystemsSequence{0, 1, 2};
  std::shared_ptr<EXP2_LogicRules> logicRules(new EXP2_LogicRules(eventTimes, subsystemsSequence));

  double startTime = 0.0;
  double finalTime = 3.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(1.0);
  partitioningTimes.push_back(2.0);
  partitioningTimes.push_back(finalTime);

  EXP2_System::state_vector_t initState(2.0, 3.0);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout
  EXP2_System systemDynamics(logicRules);
  TimeTriggeredRollout<STATE_DIM, INPUT_DIM> timeTriggeredRollout(systemDynamics);

  // system derivatives
  EXP2_SystemDerivative systemDerivative(logicRules);

  // system constraints
  EXP2_constraint systemConstraint(logicRules);

  // system cost functions
  EXP2_CostFunction systemCostFunction(logicRules);

  // system operatingTrajectories
  Eigen::Matrix<double, STATE_DIM, 1> stateOperatingPoint = Eigen::Matrix<double, STATE_DIM, 1>::Zero();
  Eigen::Matrix<double, INPUT_DIM, 1> inputOperatingPoint = Eigen::Matrix<double, INPUT_DIM, 1>::Zero();
  EXP2_SystemOperatingTrajectories operatingTrajectories(stateOperatingPoint, inputOperatingPoint);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/

  // SLQ - single core version
  SLQ<STATE_DIM, INPUT_DIM> slq(&timeTriggeredRollout, &systemDerivative, &systemConstraint, &systemCostFunction, &operatingTrajectories,
                                slqSettings, logicRules);

  // run single core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  }
  slq.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  SLQ<STATE_DIM, INPUT_DIM>::primal_solution_t solution = slq.primalSolution(finalTime);

  // get performance indices
  double totalCost, totalCost_mp;
  double constraint1ISE, constraint1ISE_mp;
  double constraint2ISE, constraint2ISE_mp;
  slq.getPerformanceIndeces(totalCost, constraint1ISE, constraint2ISE);
  slq.getPerformanceIndeces(totalCost_mp, constraint1ISE_mp, constraint2ISE_mp);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
