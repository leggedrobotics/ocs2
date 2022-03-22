/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_oc/rollout/RolloutSettings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>

#include "ocs2_oc/test/ball_dynamics_staterollout.h"
#include "ocs2_oc/test/dynamics_hybrid_slq_test.h"
#include "ocs2_oc/test/pendulum_dynamics_staterollout.h"

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;
using scalar_array_t = ocs2::scalar_array_t;
using size_array_t = ocs2::size_array_t;
using vector_array_t = ocs2::vector_array_t;
using matrix_array_t = ocs2::matrix_array_t;
using time_interval_t = std::pair<scalar_t, scalar_t>;
using time_interval_array_t = std::vector<time_interval_t>;

/*
 *     Test 1 for StateTriggeredRollout
 *     The system being tested is a bouncing ball system with dissipation on bouncing
 *
 *     Guard Surfaces are: x_1 > 0
 *                         x_1 < 0.5
 *
 *     The following tests are implemented and performed:
 *       - No penetration of Guard Surfaces.
 *       - Conservation of energy in between jumps.
 *       - Event times compared to accurate run of rollout.
 */
TEST(StateRolloutTests, rolloutTestBallDynamics) {
  const size_t nx = 2;
  const size_t nu = 1;

  // Construct State TriggerdRollout Object
  ocs2::rollout::Settings rolloutSettings;
  rolloutSettings.absTolODE = 1e-10;
  rolloutSettings.relTolODE = 1e-7;
  rolloutSettings.timeStep = 1e-3;
  ocs2::ballDyn dynamics;
  ocs2::StateTriggeredRollout rollout(dynamics, rolloutSettings);
  // Construct Variables for run
  // Simulation time
  scalar_t t0 = 0;
  scalar_t t1 = 10;
  // Initial State
  vector_t initState(nx);
  initState << 1, 0;
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);

  vector_t bias(nu);
  bias << 0;
  vector_array_t biasArray(1, bias);

  matrix_t gain(nu, nx);
  gain << 0, 0;
  matrix_array_t gainArray(1, gain);
  ocs2::LinearController control(timestamp, biasArray, gainArray);

  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  vector_array_t stateTrajectory(0);
  vector_array_t inputTrajectory(0);
  ocs2::ModeSchedule modeSchedule;
  // Output State
  vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, &control, modeSchedule, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  scalar_t energyLast = 9.81 * stateTrajectory[0][0] + +0.5 * stateTrajectory[0][1] * stateTrajectory[0][1];
  size_t eventCounter = 0;

  for (int i = 0; i < timeTrajectory.size(); i++) {
    // Test 1: Energy Conservation
    scalar_t energy = 9.81 * stateTrajectory[i][0] + 0.5 * stateTrajectory[i][1] * stateTrajectory[i][1];
    if (i != eventsPastTheEndIndeces[eventCounter]) {
      EXPECT_NEAR(energy, energyLast, 1e-5);
    } else {
      eventCounter++;
    }
    energyLast = energy;
    // Test 2a: No Significant penetration of first Guard Surface
    EXPECT_GT(stateTrajectory[i][0], -1e-6);
    // Test 2b: No Significant penetration of second Guard Surface, after first event
    if (i > eventsPastTheEndIndeces[0]) {
      EXPECT_GT(-stateTrajectory[i][0] + 0.5, -1e-6);
    }
  }
  // Test 3: Event times (due to high number of actual event times)
  const scalar_array_t eventTestTimes{0.451523642, 0.594011465, 0.74139258,  0.901399703, 1.06631179,  1.24745334,  1.433332390,
                                      2.130901210, 2.79359158,  3.423147440, 4.02122550,  4.58939966,  5.129165110, 5.641942290,
                                      6.12908061,  6.591862020, 7.03150435,  7.44916457,  7.845941770, 8.222880120, 8.58097155,
                                      8.921158410, 9.24433592,  9.55135456,  9.84302227};

  ASSERT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    EXPECT_NEAR(timeTrajectory[eventsPastTheEndIndeces[i] - 1], eventTestTimes[i], 1e-6);
  }

  EXPECT_EQ(modeSchedule.eventTimes.size() + 1, modeSchedule.modeSequence.size());
  ASSERT_EQ(eventTestTimes.size(), modeSchedule.eventTimes.size());
  for (int i = 0; i < eventTestTimes.size(); i++) {
    EXPECT_NEAR(eventTestTimes[i], modeSchedule.eventTimes[i], 1e-6);
  }
}
/*
 * 		Test 2 for StateTriggeredRollout
 * 		The system being tested is a pendulum system with an object on which it bounces
 * 		the bounces are dissipative
 *
 * 		Guard Surfaces are:			x[0] > 0
 *
 * 		The following tests are implemented and performed:
 *
 * 		-	No penetration of Guard Surface
 * 		- 	Conservation of energy in between jumps
 * 		- 	Eventtimes compared to accurate run of rollout
 */
TEST(StateRolloutTests, rolloutTestPendulumDynamics) {
  const size_t nx = 2;
  const size_t nu = 1;

  // Construct State TriggerdRollout Object
  ocs2::rollout::Settings rolloutSettings;
  ocs2::pendulum_dyn dynamics;
  ocs2::StateTriggeredRollout rollout(dynamics, rolloutSettings);
  // Construct Variables for run
  // Simulation time
  scalar_t t0 = 0;
  scalar_t t1 = 15;
  // Initial State
  vector_t initState(nx);
  initState << 3.1415, 0;
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);
  // bias Array of Controller
  vector_t bias(nu);
  bias << 0;
  vector_array_t biasArray(1, bias);
  // gain Array of Controller
  matrix_t gain(nu, nx);
  gain << 0, 0;
  matrix_array_t gainArray(1, gain);

  ocs2::LinearController control(timestamp, biasArray, gainArray);

  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  vector_array_t stateTrajectory(0);
  vector_array_t inputTrajectory(0);
  ocs2::ModeSchedule modeSchedule;
  // Output State
  vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, &control, modeSchedule, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  scalar_t energyPrevious = 9.81 * 2;  // Initial energy (pendulum in upright position h = 2*L)
  size_t eventCounter = 0;

  // Test State Trajectory
  for (int i = 0; i < timeTrajectory.size(); i++) {
    // Test 1: No Significant penetration of Guard Surface
    EXPECT_GT(stateTrajectory[i][0], -1e-6);
    // Test 2: No Significant loss of energy along trajectory (apart from due to damping during bounce)
    scalar_t h = 1 - std::cos(stateTrajectory[i][0]);                       // height at time i (since length = 1)
    scalar_t vx = stateTrajectory[i][1] * std::cos(stateTrajectory[i][0]);  // x component velocity
    scalar_t vy = stateTrajectory[i][1] * std::sin(stateTrajectory[i][0]);  // y component velocity
    scalar_t vsq = std::pow(vx, 2) + std::pow(vy, 2);                       // squared velocity
    scalar_t E = 9.81 * h + 0.5 * vsq;

    if (i != eventsPastTheEndIndeces[eventCounter]) {
      EXPECT_NEAR(E, energyPrevious, 1e-5);
    } else {
      eventCounter++;
    }
    energyPrevious = E;
  }

  // Test 3: Event times
  const scalar_array_t eventTestTimes{3.62891045, 5.0851566,  6.37201902, 7.57356039, 8.72247948,
                                      9.83580023, 10.9237331, 11.9929386, 13.0480082, 14.0922308};
  ASSERT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    EXPECT_NEAR(timeTrajectory[eventsPastTheEndIndeces[i] - 1], eventTestTimes[i], 1e-6);
  }

  EXPECT_EQ(modeSchedule.eventTimes.size() + 1, modeSchedule.modeSequence.size());
  ASSERT_EQ(eventTestTimes.size(), modeSchedule.eventTimes.size());
  for (int i = 0; i < eventTestTimes.size(); i++) {
    EXPECT_NEAR(eventTestTimes[i], modeSchedule.eventTimes[i], 1e-6);
  }
}
/*
 * 		Test 3 for StateTriggeredRollout
 * 		The system being tested is a combination of two Linear system
 * 		it switches between these systems on an event
 *
 * 		Guard Surfaces are:			x[0]*x[1] < 0 			when in mode 0
 * 									x[0]*x[1] > 0 			when in mode 1
 *
 * 		The following tests are implemented and performed:
 *
 * 		-	No penetration of Guard Surface
 * 		- 	Eventtimes compared to accurate run of rollout
 */
TEST(StateRolloutTests, runHybridDynamics) {
  const size_t nx = 3;
  const size_t nu = 1;

  // Construct State TriggerdRollout Object
  ocs2::rollout::Settings rolloutSettings;
  ocs2::HybridSysDynamics dynamics;
  ocs2::StateTriggeredRollout rollout(dynamics, rolloutSettings);
  // Construct Variables for run
  // Simulation time
  const scalar_t t0 = 0;
  const scalar_t t1 = 5;
  // Initial State
  vector_t initState(nx);
  initState << 5, 2, 1;
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);
  vector_t bias(nu);
  bias << 0;
  vector_array_t biasArray(1, bias);

  matrix_t gain(nu, nx);
  gain << 0, 0, 0;
  matrix_array_t gainArray(1, gain);
  ocs2::LinearController control(timestamp, biasArray, gainArray);
  ocs2::ModeSchedule modeSchedule;
  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  vector_array_t stateTrajectory(0);
  vector_array_t inputTrajectory(0);

  // Output State
  vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, &control, modeSchedule, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  // Test (and display statetrajectory)
  for (int i = 0; i < timeTrajectory.size(); i++) {
    // Test 1: No Significant penetration of Guard Surface
    scalar_t currentTime = timeTrajectory[i];
    vector_t currentState = stateTrajectory[i];
    vector_t currentGuardValues = dynamics.computeGuardSurfaces(currentTime, currentState);

    EXPECT_GT(currentGuardValues[0], -1e-6);
    EXPECT_GT(currentGuardValues[1], -1e-6);
  }

  // Test 3: Event times
  const scalar_array_t eventTestTimes{0.126835459, 1.73439091, 2.25798967, 3.86554513, 4.38914388};
  ASSERT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    EXPECT_NEAR(timeTrajectory[eventsPastTheEndIndeces[i] - 1], eventTestTimes[i], 1e-8);
  }

  EXPECT_EQ(modeSchedule.eventTimes.size() + 1, modeSchedule.modeSequence.size());
  ASSERT_EQ(eventTestTimes.size(), modeSchedule.eventTimes.size());
  for (int i = 0; i < eventTestTimes.size(); i++) {
    EXPECT_NEAR(eventTestTimes[i], modeSchedule.eventTimes[i], 1e-6);
  }
}
