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

#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>

#include "ocs2_ddp/test/bouncingmass/OverallReference.h"
#include "ocs2_ddp/test/bouncingmass/SystemModel.h"

/*	Test for StateTriggeredRollout in combination with SLQ
 *
 * The system being tested is a system consisting of a mass and a wall
 * the mass bounces against the wall. The control objective is to follow a predefined
 * trajectory consisting of fifth order polynomial.
 *
 * The system is identical to the example presented in
 * [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 *
 * Guard Surfaces are:    x[0] > 0
 *
 * Cost function is:      x(t)^T Q x(t) + u(t)^T R u(t) + x(t1)^T P x(t1)^T
 *                        Q = [50,   0;
 *                             0,  50];
 *                        P = [56.63, 7.07;
 *                             7.07, 8.01];
 *                        R = 1;
 *
 * Initial controller is: K = [25,10]
 *
 * The following tests are implemented and performed:
 * (1) No penetration of Guard Surfaces
 * (2) Check of cost function compared against cost calculated during trusted run of SLQ
 */
TEST(BouncingMassTest, DISABLED_state_rollout_slq) {
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using matrix_t = ocs2::matrix_t;
  using scalar_array_t = ocs2::scalar_array_t;
  using vector_array_t = ocs2::vector_array_t;
  using matrix_array_t = ocs2::matrix_array_t;

  ocs2::ddp::Settings ddpSettings;
  ddpSettings.algorithm_ = ocs2::ddp::Algorithm::SLQ;
  ddpSettings.displayInfo_ = false;
  ddpSettings.displayShortSummary_ = true;
  ddpSettings.maxNumIterations_ = 30;
  ddpSettings.minRelCost_ = 1e-4;
  ddpSettings.nThreads_ = 1;
  ddpSettings.checkNumericalStability_ = true;
  ddpSettings.absTolODE_ = 1e-10;
  ddpSettings.relTolODE_ = 1e-7;
  ddpSettings.maxNumStepsPerSecond_ = 10000;
  ddpSettings.useFeedbackPolicy_ = true;
  ddpSettings.debugPrintRollout_ = false;

  ocs2::rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-10;
  rolloutSettings.relTolODE = 1e-7;
  rolloutSettings.timeStep = 1e-3;
  rolloutSettings.maxNumStepsPerSecond = 10000;
  rolloutSettings.maxSingleEventIterations = 5;
  rolloutSettings.useTrajectorySpreadingController = true;

  // Parameters
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 2.5;
  const vector_t x0 = Eigen::Matrix<scalar_t, 3, 1>(0.7, 0.5, 0);
  const vector_t u0 = vector_t::Zero(INPUT_DIM);
  bool outputSolution = false;

  // Generation of Reference Trajectory
  const scalar_array_t trajTimes{0, 0.2, 0.8, 1.0, 1.2, 1.8, 2.0};

  vector_t state0(STATE_DIM);  //	Initial and final state
  state0 << 0.5, 0, 0;
  vector_t state1(STATE_DIM);  //	Hard impact
  state1 << 0, -5, 0;
  vector_t state2(STATE_DIM);  // Soft impact
  state2 << 0, -1, 0;

  const scalar_t delta = 0.5;
  const vector_array_t trajStates{state0, state1, state2, state2, state1, state2, state0};
  OverallReference reference(trajTimes, trajStates);
  reference.extendref(delta);

  // Dynamics, Constraints and derivative classes
  BouncingMassDynamics systemDynamics;

  // Cost Function
  matrix_t Q(STATE_DIM, STATE_DIM);
  Q << 50.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0;
  matrix_t R(INPUT_DIM, INPUT_DIM);
  R << 1.0;
  std::unique_ptr<ocs2::StateInputCost> cost(new BouncingMassCost(reference, Q, R));

  matrix_t Qf(STATE_DIM, STATE_DIM);
  Qf << 56.63, 7.07, 0.0, 7.07, 8.01, 0.0, 0.0, 0.0, 0.0;
  std::unique_ptr<ocs2::StateCost> finalCost(new BouncingMassFinalCost(reference, Qf, finalTime));
  std::unique_ptr<ocs2::StateCost> preJumpCost(new BouncingMassFinalCost(reference, Qf, finalTime));

  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr.reset(systemDynamics.clone());
  problem.costPtr->add("cost", std::move(cost));
  problem.preJumpCostPtr->add("preJumpCost", std::move(preJumpCost));
  problem.finalCostPtr->add("finalCost", std::move(finalCost));

  // Rollout Class
  ocs2::StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // Initial Controller
  matrix_t controllerGain(INPUT_DIM, STATE_DIM);
  controllerGain << 25, 10, 0;
  vector_t controllerBias;

  matrix_array_t controllerGainArray;
  vector_array_t controllerBiasArray;
  scalar_array_t timeStampArray;

  constexpr scalar_t controllerDeltaTime = 1e-3;  // Time step for controller time array
  constexpr scalar_t eps = ocs2::numeric_traits::weakEpsilon<scalar_t>();
  scalar_array_t controlTimes = trajTimes;
  controlTimes.push_back(finalTime);

  for (int i = 0; i < controlTimes.size() - 1; i++) {
    scalar_t timeSteps = (controlTimes[i + 1] + eps - controlTimes[i]) / controllerDeltaTime;
    scalar_t timeStamp = 0;
    vector_t refState;

    for (int j = 0; j <= timeSteps; j++) {
      if (j == 0 && i < controlTimes.size() - 2) {
        timeStamp = controlTimes[i] + eps;
      } else {
        timeStamp = controlTimes[i] + j * controllerDeltaTime;
      }

      vector_t refState = reference.getState(timeStamp);
      vector_t refInput = reference.getInput(timeStamp);
      controllerBias = controllerGain * refState + refInput;

      timeStampArray.push_back(timeStamp);
      controllerGainArray.push_back(-controllerGain);
      controllerBiasArray.push_back(controllerBias);
    }
  }

  ocs2::LinearController initController(timeStampArray, controllerBiasArray, controllerGainArray);

  ocs2::OperatingPoints operatingTrajectories(x0, u0);
  // SLQ
  ocs2::SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.run(startTime, x0, finalTime, &initController);
  auto solutionST = slq.primalSolution(finalTime);

  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    // Test 1: No penetration of Guard Surfaces
    EXPECT_GT(solutionST.stateTrajectory_[i][0], -ddpSettings.absTolODE_);
    // Display output
    // format: idx;time;x[0];xref[0];x[1];xref[1];u;uref
    if (outputSolution) {
      auto idx = static_cast<int>(solutionST.stateTrajectory_[i][2]);

      vector_t uRef = reference.getInput(solutionST.timeTrajectory_[i]);
      vector_t xRef = reference.getState(idx, solutionST.timeTrajectory_[i]);

      std::cerr << i << ";" << idx << ";";
      std::cerr << std::setprecision(25) << solutionST.timeTrajectory_[i];
      std::cerr << std::setprecision(6) << ";" << solutionST.stateTrajectory_[i][0] << ";" << xRef[0] << ";";
      std::cerr << solutionST.stateTrajectory_[i][1] << ";" << xRef[1] << ";";
      std::cerr << solutionST.inputTrajectory_[i][0] << ";" << uRef[0] << std::endl;
    }
  }

  // Test 2: Check of cost function
  auto performanceIndeces = slq.getPerformanceIndeces();
  constexpr scalar_t expectedCost = 7.15;
  EXPECT_LT(std::fabs(performanceIndeces.cost - expectedCost), 100 * ddpSettings.minRelCost_);
}
