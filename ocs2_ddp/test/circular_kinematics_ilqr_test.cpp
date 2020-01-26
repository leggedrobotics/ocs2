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
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/test/circular_kinematics.h>

using namespace ocs2;

enum { STATE_DIM = 2, INPUT_DIM = 2 };

// TEST(exp0_ilqr_test, exp0_ilqr_test) {
void testFunction() {
  using ilqr_t = ILQR<STATE_DIM, INPUT_DIM>;

  ILQR_Settings ilqrSettings;
  ilqrSettings.ddpSettings_.displayInfo_ = true;
  ilqrSettings.ddpSettings_.displayShortSummary_ = true;
  ilqrSettings.ddpSettings_.absTolODE_ = 1e-10;
  ilqrSettings.ddpSettings_.relTolODE_ = 1e-7;
  ilqrSettings.ddpSettings_.maxNumStepsPerSecond_ = 1000000;
  ilqrSettings.ddpSettings_.maxNumIterations_ = 30;
  ilqrSettings.ddpSettings_.noStateConstraints_ = true;
  ilqrSettings.ddpSettings_.minRelCost_ = 5e-4;
  ilqrSettings.ddpSettings_.checkNumericalStability_ = false;
  ilqrSettings.ddpSettings_.useFeedbackPolicy_ = true;
  ilqrSettings.ddpSettings_.debugPrintRollout_ = false;
  ilqrSettings.ddpSettings_.strategy_ = DDP_Strategy::LINE_SEARCH;
  ilqrSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.01;
  ilqrSettings.ddpSettings_.lineSearch_.hessianCorrectionStrategy_ = Hessian_Correction::EIGENVALUE_MODIFICATION;
  ilqrSettings.ddpSettings_.lineSearch_.hessianCorrectionMultiple_ = 1e-3;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-7;
  rolloutSettings.relTolODE_ = 1e-5;
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
  // ILQR - single-thread version
  ilqrSettings.ddpSettings_.nThreads_ = 1;
  ilqr_t slqST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ilqrSettings);

  ilqrSettings.ddpSettings_.nThreads_ = 3;
  // ILQR - multi-thread version
  ilqr_t slqMT(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ilqrSettings);

  // run single core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core ILQR" << std::endl;
  }
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // run multi-core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> multi-core ILQR" << std::endl;
  }
  //  slqMT.run(startTime, initState, finalTime, partitioningTimes);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  // get solution
  //  ilqr_t::primal_solution_t solutionST = slqST.primalSolution(finalTime);
  //  ilqr_t::primal_solution_t solutionMT = slqMT.primalSolution(finalTime);
  //
  //  // get performance indices
  //  double totalCostST, totalCostMT;
  //  double constraint1ISE_ST, constraint1ISE_MT;
  //  double constraint2ISE_ST, constraint2ISE_MT;
  //  slqST.getPerformanceIndeces(totalCostST, constraint1ISE_ST, constraint2ISE_ST);
  //  slqMT.getPerformanceIndeces(totalCostMT, constraint1ISE_MT, constraint2ISE_MT);

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
}

void testFunctionGoodInit() {
  using ilqr_t = ILQR<STATE_DIM, INPUT_DIM>;
  using slq_t = SLQ<STATE_DIM, INPUT_DIM>;

  SLQ_Settings slqSettings;
  slqSettings.preComputeRiccatiTerms_ = false;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.RiccatiIntegratorType_ = IntegratorType::ODE45;
  slqSettings.ddpSettings_.displayInfo_ = true;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-7;
  slqSettings.ddpSettings_.relTolODE_ = 1e-5;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.maxNumIterations_ = 150;
  slqSettings.ddpSettings_.noStateConstraints_ = true;
  slqSettings.ddpSettings_.minRelCost_ = 1e-3;
  slqSettings.ddpSettings_.checkNumericalStability_ = false;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;
  slqSettings.ddpSettings_.strategy_ = DDP_Strategy::LINE_SEARCH;
  slqSettings.ddpSettings_.lineSearch_.minStepLength_ = 0.01;
  slqSettings.ddpSettings_.lineSearch_.hessianCorrectionStrategy_ = Hessian_Correction::CHOLESKY_MODIFICATION;
  slqSettings.ddpSettings_.lineSearch_.hessianCorrectionMultiple_ = 1e-3;

  Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-7;
  rolloutSettings.relTolODE_ = 1e-5;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;

  ILQR_Settings ilqrSettings;
  ilqrSettings.ddpSettings_ = slqSettings.ddpSettings_;

  double startTime = 0.0;
  double finalTime = 10.0;

  // partitioning times
  std::vector<double> partitioningTimes;
  partitioningTimes.push_back(startTime);
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
  slqSettings.ddpSettings_.maxNumIterations_ = 2;
  slq_t slqST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, slqSettings);

  ilqrSettings.ddpSettings_.nThreads_ = 1;
  // ILQR - single-thread version
  ilqr_t ilqrST(&timeTriggeredRollout, &systemDynamics, &systemConstraint, &systemCostFunction, &operatingTrajectories, ilqrSettings);

  // run single core SLQ
  if (slqSettings.ddpSettings_.displayInfo_ || slqSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core SLQ" << std::endl;
  }
  slqST.run(startTime, initState, finalTime, partitioningTimes);

  // get solution
  slq_t::primal_solution_t solutionST = slqST.primalSolution(finalTime);

  // run single core ILQR
  if (ilqrSettings.ddpSettings_.displayInfo_ || ilqrSettings.ddpSettings_.displayShortSummary_) {
    std::cerr << "\n>>> single-core ILQR" << std::endl;
  }
  ilqr_t::controller_ptr_array_t initController{solutionST.controllerPtr_.get()};
  ilqrST.run(startTime, initState, finalTime, partitioningTimes, initController);
}

int main(int argc, char** argv) {
  //  testing::InitGoogleTest(&argc, argv);
  //  return RUN_ALL_TESTS();
  testFunctionGoodInit();
}
