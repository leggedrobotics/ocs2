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

/*
 * Test for State-Triggered hybrid SLQ
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
TEST(BouncingMassTest, state_triggered_hybrid_slq) {
  // Parameters
  constexpr bool outputSolution = false;
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 2.5;
  const ocs2::vector_t x0 = Eigen::Matrix<ocs2::scalar_t, 3, 1>(0.7, 0.5, 0);
  const ocs2::vector_t u0 = ocs2::vector_t::Zero(INPUT_DIM);

  const auto ddpSettings = [&]() {
    ocs2::ddp::Settings s;
    s.algorithm_ = ocs2::ddp::Algorithm::SLQ;
    s.displayInfo_ = false;
    s.displayShortSummary_ = true;
    s.maxNumIterations_ = 30;
    s.minRelCost_ = 1e-4;
    s.nThreads_ = 1;
    s.checkNumericalStability_ = true;
    s.absTolODE_ = 1e-10;
    s.relTolODE_ = 1e-7;
    s.maxNumStepsPerSecond_ = 10000;
    s.useFeedbackPolicy_ = true;
    s.debugPrintRollout_ = false;
    return s;
  }();

  const auto rolloutSettings = [&]() {
    ocs2::rollout::Settings s;
    s.absTolODE = 1e-10;
    s.relTolODE = 1e-7;
    s.timeStep = 1e-3;
    s.maxNumStepsPerSecond = 10000;
    s.maxSingleEventIterations = 5;
    s.useTrajectorySpreadingController = true;
    return s;
  }();

  // Generation of Reference Trajectory
  const ocs2::scalar_array_t trajTimes{0, 0.2, 0.8, 1.0, 1.2, 1.8, 2.0};
  const ocs2::vector_t state0 = (ocs2::vector_t(STATE_DIM) << 0.5, 0, 0).finished();  // Initial and final state
  const ocs2::vector_t state1 = (ocs2::vector_t(STATE_DIM) << 0, -5, 0).finished();   // Hard impact
  const ocs2::vector_t state2 = (ocs2::vector_t(STATE_DIM) << 0, -1, 0).finished();   // Soft impact
  const ocs2::vector_array_t trajStates{state0, state1, state2, state2, state1, state2, state0};

  OverallReference reference(trajTimes, trajStates);
  constexpr ocs2::scalar_t delta = 0.5;
  reference.extendref(delta);

  // OCP
  ocs2::OptimalControlProblem problem;

  // Dynamics, Constraints and derivative classes
  BouncingMassDynamics systemDynamics;
  problem.dynamicsPtr.reset(systemDynamics.clone());

  // Cost Function
  const ocs2::matrix_t Q = (ocs2::matrix_t(STATE_DIM, STATE_DIM) << 50.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0).finished();
  const ocs2::matrix_t R = (ocs2::matrix_t(INPUT_DIM, INPUT_DIM) << 1.0).finished();
  const ocs2::matrix_t Qf = (ocs2::matrix_t(STATE_DIM, STATE_DIM) << 56.63, 7.07, 0.0, 7.07, 8.01, 0.0, 0.0, 0.0, 0.0).finished();
  problem.costPtr->add("cost", std::make_unique<BouncingMassCost>(reference, Q, R));
  problem.preJumpCostPtr->add("preJumpCost", std::make_unique<BouncingMassFinalCost>(reference, Qf, finalTime));
  problem.finalCostPtr->add("finalCost", std::make_unique<BouncingMassFinalCost>(reference, Qf, finalTime));

  // Rollout Class
  ocs2::StateTriggeredRollout stateTriggeredRollout(systemDynamics, rolloutSettings);

  // Initial Controller
  const auto initController = [&]() {
    ocs2::matrix_array_t controllerGainArray;
    ocs2::vector_array_t controllerBiasArray;
    ocs2::scalar_array_t timeStampArray;

    constexpr ocs2::scalar_t controllerDeltaTime = 1e-3;  // Time step for controller time array
    constexpr ocs2::scalar_t eps = ocs2::numeric_traits::weakEpsilon<ocs2::scalar_t>();
    ocs2::scalar_array_t controlTimes = trajTimes;
    controlTimes.push_back(finalTime);

    const ocs2::matrix_t controllerGain = (ocs2::matrix_t(INPUT_DIM, STATE_DIM) << 25, 10, 0).finished();
    for (int i = 0; i < controlTimes.size() - 1; i++) {
      const ocs2::scalar_t timeSteps = (controlTimes[i + 1] + eps - controlTimes[i]) / controllerDeltaTime;
      ocs2::scalar_t timeStamp = 0;
      for (int j = 0; j <= timeSteps; j++) {
        if (j == 0 && i < controlTimes.size() - 2) {
          timeStamp = controlTimes[i] + eps;
        } else {
          timeStamp = controlTimes[i] + j * controllerDeltaTime;
        }

        ocs2::vector_t refState = reference.getState(timeStamp);
        ocs2::vector_t refInput = reference.getInput(timeStamp);
        ocs2::vector_t controllerBias = controllerGain * refState + refInput;

        timeStampArray.push_back(timeStamp);
        controllerGainArray.push_back(-controllerGain);
        controllerBiasArray.push_back(controllerBias);
      }
    }

    return ocs2::LinearController(timeStampArray, controllerBiasArray, controllerGainArray);
  }();

  // operating point
  const ocs2::OperatingPoints operatingTrajectories(x0, u0);

  // SLQ
  ocs2::SLQ slq(ddpSettings, stateTriggeredRollout, problem, operatingTrajectories);
  slq.run(startTime, x0, finalTime, &initController);
  const auto solutionST = slq.primalSolution(finalTime);

  // Test 1: No penetration of Guard Surfaces
  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    EXPECT_GT(solutionST.stateTrajectory_[i](0), -ddpSettings.absTolODE_);

    // Display output: format: idx;time;x[0];xref[0];x[1];xref[1];u;uref
    if (outputSolution) {
      const auto idx = static_cast<int>(solutionST.stateTrajectory_[i][2]);
      const auto uRef = reference.getInput(solutionST.timeTrajectory_[i]);
      const auto xRef = reference.getState(idx, solutionST.timeTrajectory_[i]);
      std::cerr << i << ";" << idx << ";";
      std::cerr << std::setprecision(25) << solutionST.timeTrajectory_[i];
      std::cerr << std::setprecision(6) << ";" << solutionST.stateTrajectory_[i](0) << ";" << xRef[0] << ";";
      std::cerr << solutionST.stateTrajectory_[i](1) << ";" << xRef[1] << ";";
      std::cerr << solutionST.inputTrajectory_[i](0) << ";" << uRef[0] << std::endl;
    }
  }  // end of i loop

  // Test 2: Check of cost function
  constexpr ocs2::scalar_t expectedCost = 7.15;
  const auto performanceIndeces = slq.getPerformanceIndeces();
  EXPECT_LE(performanceIndeces.cost, expectedCost);
}
