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

#include <ocs2_ddp/ILQR.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/circular_kinematics.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 2 };

TEST(circular_kinematics_ilqr_test, circular_kinematics_ilqr_test) {
  ILQR_Settings ilqrSettings;
  ilqrSettings.ddpSettings_.displayInfo_ = false;
  ilqrSettings.ddpSettings_.displayShortSummary_ = true;
  ilqrSettings.ddpSettings_.checkNumericalStability_ = false;
  ilqrSettings.ddpSettings_.debugPrintRollout_ = false;
  ilqrSettings.ddpSettings_.absTolODE_ = 1e-9;
  ilqrSettings.ddpSettings_.relTolODE_ = 1e-7;
  ilqrSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  ilqrSettings.ddpSettings_.maxNumIterations_ = 150;
  ilqrSettings.ddpSettings_.minRelCost_ = 1e-3;
  ilqrSettings.ddpSettings_.constraintTolerance_ = 1e-5;
  ilqrSettings.ddpSettings_.constraintPenaltyInitialValue_ = 2.0;
  ilqrSettings.ddpSettings_.constraintPenaltyIncreaseRate_ = 1.5;
  ilqrSettings.ddpSettings_.strategy_ = ddp_strategy::type::LINE_SEARCH;
  ilqrSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.01;
  ilqrSettings.ddpSettings_.lineSearch_.hessianCorrectionStrategy_ = hessian_correction::Strategy::CHOLESKY_MODIFICATION;
  ilqrSettings.ddpSettings_.lineSearch_.hessianCorrectionMultiple_ = 1e-3;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-9;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  scalar_t startTime = 0.0;
  scalar_t finalTime = 10.0;

  // partitioning times
  std::vector<scalar_t> partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime / 2.0);
  partitioningTimes.push_back(finalTime);

  vector_t initState(2);
  initState << 1.0, 0.0;  // radius 1.0

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // system rollout and system derivatives
  CircularKinematicsSystem systemDynamics;
  TimeTriggeredRollout timeTriggeredRollout(systemDynamics, rolloutSettings);

  // cost functions
  CircularKinematicsCost systemCostFunction;

  boost::filesystem::path filePath(__FILE__);
  std::string libraryFolder = filePath.parent_path().generic_string() + "/ddp_test_generated";
  systemCostFunction.initialize("circular_kinematics_cost", libraryFolder, true, true);

  // system constraints
  CircularKinematicsConstraints systemConstraint;

  // system operatingTrajectories
  OperatingPoints operatingTrajectories(initState, vector_t::Zero(INPUT_DIM));

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // ILQR - single-thread version
  ilqrSettings.ddpSettings_.nThreads_ = 1;
  ILQR ilqrST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ilqrSettings);

  // ILQR - multi-thread version
  ilqrSettings.ddpSettings_.nThreads_ = 3;
  ILQR ilqrMT(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ilqrSettings);

  // run single core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core ILQR" << std::endl;
  }
  ilqrST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> multi-core ILQR" << std::endl;
  }
  ilqrMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  auto solutionST = ilqrST.primalSolution(finalTime);
  auto solutionMT = ilqrMT.primalSolution(finalTime);

  // get performance indices
  auto performanceIndecesST = ilqrST.getPerformanceIndeces();
  auto performanceIndecesMT = ilqrMT.getPerformanceIndeces();

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const scalar_t expectedCost = 0.1;
  ASSERT_LT(performanceIndecesST.totalCost - expectedCost, 0.0)
      << "MESSAGE: single-threaded SLQ failed in the Circular_Kinematics's cost test!";
  ASSERT_LT(performanceIndecesMT.totalCost - expectedCost, 0.0)
      << "MESSAGE: multi-threaded SLQ failed in the Circular_Kinematics's cost test!";

  const scalar_t expectedISE1 = 0.0;
  ASSERT_LT(fabs(performanceIndecesST.stateInputEqConstraintISE - expectedISE1), ilqrSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: single-threaded SLQ failed in the Circular_Kinematics's type-1 constraint ISE test!";
  ASSERT_LT(fabs(performanceIndecesMT.stateInputEqConstraintISE - expectedISE1), ilqrSettings.ddpSettings_.constraintTolerance_)
      << "MESSAGE: multi-threaded SLQ failed in the Circular_Kinematics's type-1 constraint ISE test!";
}
