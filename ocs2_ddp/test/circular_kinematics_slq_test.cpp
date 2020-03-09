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
#include <string>

#include <boost/filesystem.hpp>

#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/circular_kinematics.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 2 };

TEST(circular_kinematics_slq_test, circular_kinematics_slq_test) {
  using slq_t = SLQ<STATE_DIM, INPUT_DIM>;

  SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.preComputeRiccatiTerms_ = false;
  slqSettings.RiccatiIntegratorType_ = IntegratorType::ODE45;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;
  slqSettings.ddpSettings_.absTolODE_ = 1e-9;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.maxNumIterations_ = 150;
  slqSettings.ddpSettings_.minRelCost_ = 1e-3;
  slqSettings.ddpSettings_.constraintTolerance_ = 1e-5;
  slqSettings.ddpSettings_.constraintPenaltyInitialValue_ = 2.0;
  slqSettings.ddpSettings_.constraintPenaltyIncreaseRate_ = 2.0;
  slqSettings.ddpSettings_.strategy_ = ddp_strategy::type::LINE_SEARCH;
  slqSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.01;
  slqSettings.ddpSettings_.lineSearch_.hessianCorrectionStrategy_ = hessian_correction::Strategy::CHOLESKY_MODIFICATION;
  slqSettings.ddpSettings_.lineSearch_.hessianCorrectionMultiple_ = 1e-3;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-9;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  double startTime = 0.0;
  double finalTime = 10.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime / 2.0);
  partitioningTimes.push_back(finalTime);

  using state_vector_t = CircularKinematicsSystem::state_vector_t;
  using input_vector_t = CircularKinematicsSystem::input_vector_t;
  CircularKinematicsSystem::state_vector_t initState(1.0, 0.0);  // radius 1.0

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout and system derivatives
  CircularKinematicsSystem systemDynamics;
  TimeTriggeredRollout<STATE_DIM, INPUT_DIM> timeTriggeredRollout(systemDynamics, rolloutSettings);

  // cost functions
  CircularKinematicsCost systemCostFunction;

  boost::filesystem::path filePath(__FILE__);
  std::string libraryFolder = filePath.parent_path().generic_string() + "/ddp_test_generated";
  systemCostFunction.initialize("circular_kinematics_cost", libraryFolder, true, true);

  // system constraints
  CircularKinematicsConstraints systemConstraint;

  // system operatingTrajectories
  state_vector_t stateOperatingPoint = initState;
  CircularKinematicsSystemOperatingTrajectories operatingTrajectories(initState, input_vector_t::Zero());

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // SLQ - single-thread version
  slqSettings.ddpSettings_.nThreads_ = 1;
  slq_t slqST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, slqSettings);

  // SLQ - multi-thread version
  slqSettings.ddpSettings_.nThreads_ = 3;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slq_t slqMT(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, slqSettings);
  slqMT.useParallelRiccatiSolverFromInitItr(false);

  // run single core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  }
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> multi-core SLQ" << std::endl;
  }
  slqMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  slq_t::primal_solution_t solutionST = slqST.primalSolution(finalTime);
  slq_t::primal_solution_t solutionMT = slqMT.primalSolution(finalTime);

  // get performance indices
  auto performanceIndecesST = slqST.getPerformanceIndeces();
  auto performanceIndecesMT = slqMT.getPerformanceIndeces();

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const double expectedCost = 0.1;
  ASSERT_LT(performanceIndecesST.totalCost - expectedCost, 0.0)
      << "MESSAGE: single-threaded SLQ failed in the Circular_Kinematics's cost test!";
  ASSERT_LT(performanceIndecesMT.totalCost - expectedCost, 0.0)
      << "MESSAGE: multi-threaded SLQ failed in the Circular_Kinematics's cost test!";

  const double expectedISE1 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateInputEqConstraintISE - expectedISE1), slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the Circular_Kinematics's type-1 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateInputEqConstraintISE  - expectedISE1), slqSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the Circular_Kinematics's type-1 constraint ISE test!";
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
