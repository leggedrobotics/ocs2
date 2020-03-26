#include <ocs2_core/control/LinearController.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include "include/ocs2_oc/test/ball_dynamics_staterollout.h"
#include "include/ocs2_oc/test/dynamics_hybrid_slq_test.h"
#include "include/ocs2_oc/test/pendulum_dynamics_staterollout.h"
#include "ocs2_core/Dimensions.h"

#include <gtest/gtest.h>
/*
 *      Test 1 for StateTriggeredRollout
 *      The system being tested is a bouncing ball system with dissipation on bouncing
 *
 *      Guard Surfaces are:  x_1 > 0
 *                           x_1 < 0.5
 *
 *      The following tests are implemented and performed:
 *        - No penetration of Guard Surfaces.
 *        - Conservation of energy in between jumps.
 *        - Event times compared to accurate run of rollout.
 */
TEST(StateRolloutTests, rolloutTestBallDynamics) {
  using DIMENSIONS = ocs2::Dimensions<2, 1>;
  using controller_t = ocs2::ControllerBase<2, 1>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using time_interval_t = std::pair<scalar_t, scalar_t>;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;

  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;

  using time_interval_array_t = std::vector<time_interval_t>;

  // Construct State TriggerdRollout Object
  ocs2::Rollout_Settings RolloutSettings;
  RolloutSettings.absTolODE_ = 1e-10;
  RolloutSettings.relTolODE_ = 1e-7;
  ocs2::ballDyn dynamics;
  ocs2::StateTriggeredRollout<2, 1> rollout(dynamics, RolloutSettings);
  // Construct Variables for run
  // Simulation time
  scalar_t t0 = 0;
  scalar_t t1 = 10;
  // Initial State
  state_vector_t initState(2, 0);
  initState[0] = 1;
  // Initial Event times (none)
  scalar_array_t eventTimes(1, t0);
  // Controller (time constant zero controller)
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);

  input_vector_t bias;
  bias << 0;
  input_vector_array_t biasArray(1, bias);

  input_state_matrix_t gain;
  gain << 0, 0;
  input_state_matrix_array_t gainArray(1, gain);
  ocs2::LinearController<2, 1> control(timestamp, biasArray, gainArray);
  ocs2::LinearController<2, 1>* controller = &control;

  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  state_vector_array_t stateTrajectory(0);
  input_vector_array_t inputTrajectory(0);
  // Output State
  state_vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, controller, eventTimes, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  double energyLast = 9.81 * stateTrajectory[0][0] + +0.5 * stateTrajectory[0][1] * stateTrajectory[0][1];
  size_t eventCounter = 0;

  for (int i = 0; i < timeTrajectory.size(); i++) {
    // Test 1: Energy Conservation
    double energy = 9.81 * stateTrajectory[i][0] + 0.5 * stateTrajectory[i][1] * stateTrajectory[i][1];
    if (i != eventsPastTheEndIndeces[eventCounter]) {
      EXPECT_LT(std::fabs(energy - energyLast), 1e-5);
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
  scalar_array_t eventTestTimes = {0.451523642, 0.594011465, 0.74139258,  0.901399703, 1.06631179,  1.24745334,  1.433332390,
                                   2.130901210, 2.79359158,  3.423147440, 4.02122550,  4.58939966,  5.129165110, 5.641942290,
                                   6.12908061,  6.591862020, 7.03150435,  7.44916457,  7.845941770, 8.222880120, 8.58097155,
                                   8.921158410, 9.24433592,  9.55135456,  9.84302227};

  EXPECT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    if (i < eventTestTimes.size()) {
      EXPECT_LT(std::fabs(timeTrajectory[eventsPastTheEndIndeces[i] - 1] - eventTestTimes[i]), 1e-6);
    }
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
  using DIMENSIONS = ocs2::Dimensions<2, 1>;
  using controller_t = ocs2::ControllerBase<2, 1>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using time_interval_t = std::pair<scalar_t, scalar_t>;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;

  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;

  using time_interval_array_t = std::vector<time_interval_t>;
  // Construct State TriggerdRollout Object
  ocs2::Rollout_Settings rolloutSettings;
  ocs2::pendulum_dyn dynamics;
  ocs2::StateTriggeredRollout<2, 1> rollout(dynamics, rolloutSettings);
  // Construct Variables for run
  // Simulation time
  scalar_t t0 = 0;
  scalar_t t1 = 15;
  // Initial State
  state_vector_t initState(2, 0);
  initState[0] = 3.1415;
  // Initial Event times (none)
  scalar_array_t eventTimes(1, t0);
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);
  // bias Array of Controller
  input_vector_t bias;
  bias << 0;
  input_vector_array_t biasArray(1, bias);
  // gain Array of Controller
  input_state_matrix_t gain;
  gain << 0, 0;
  input_state_matrix_array_t gainArray(1, gain);

  ocs2::LinearController<2, 1> control(timestamp, biasArray, gainArray);
  ocs2::LinearController<2, 1>* controller = &control;

  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  state_vector_array_t stateTrajectory(0);
  input_vector_array_t inputTrajectory(0);
  // Output State
  state_vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, controller, eventTimes, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

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
      EXPECT_LT(std::fabs(E - energyPrevious), 1e-5);
    } else {
      eventCounter++;
    }
    energyPrevious = E;
  }

  // Test 3: Event times
  scalar_array_t eventTestTimes = {3.62891045, 5.0851566,  6.37201902, 7.57356039, 8.72247948,
                                   9.83580023, 10.9237331, 11.9929386, 13.0480082, 14.0922308};
  EXPECT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    if (i < eventTestTimes.size()) {
      EXPECT_LT(std::fabs(timeTrajectory[eventsPastTheEndIndeces[i] - 1] - eventTestTimes[i]), 1e-6);
    }
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
  using DIMENSIONS = ocs2::Dimensions<3, 1>;
  using controller_t = ocs2::ControllerBase<3, 1>;

  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using time_interval_t = std::pair<scalar_t, scalar_t>;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;

  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using size_array_t = typename DIMENSIONS::size_array_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;

  using time_interval_array_t = std::vector<time_interval_t>;
  // Create Logic Rules
  std::vector<double> eventTimes(0);
  std::vector<size_t> subsystemsSequence{1};
  // Construct State TriggerdRollout Object
  ocs2::Rollout_Settings rolloutSettings;
  ocs2::hybridSysDynamics dynamics;
  ocs2::StateTriggeredRollout<3, 1> rollout(dynamics, rolloutSettings);
  // Construct Variables for run
  // Simulation time
  scalar_t t0 = 0;
  scalar_t t1 = 5;
  // Initial State
  state_vector_t initState(2, 0, 1);
  initState[0] = 5;
  initState[1] = 2;
  // Controller (time constant zero controller)
  scalar_array_t timestamp(1, t0);
  input_vector_t bias;
  bias << 0;
  input_vector_array_t biasArray(1, bias);

  input_state_matrix_t gain;
  gain << 0, 0, 0;
  input_state_matrix_array_t gainArray(1, gain);
  ocs2::LinearController<3, 1> control(timestamp, biasArray, gainArray);
  ocs2::LinearController<3, 1>* Controller = &control;

  // Trajectory storage
  scalar_array_t timeTrajectory(0);
  size_array_t eventsPastTheEndIndeces(0);
  state_vector_array_t stateTrajectory(0);
  input_vector_array_t inputTrajectory(0);
  // Output State
  state_vector_t finalState;
  // Run
  finalState =
      rollout.run(t0, initState, t1, Controller, eventTimes, timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  // Test (and display statetrajectory)
  for (int i = 0; i < timeTrajectory.size(); i++) {
    // Test 1: No Significant penetration of Guard Surface
    dynamic_vector_t currentGuardValues;
    scalar_t currentTime = timeTrajectory[i];
    state_vector_t currentState = stateTrajectory[i];
    dynamics.computeGuardSurfaces(currentTime, currentState, currentGuardValues);

    EXPECT_GT(currentGuardValues[0], -1e-6);
    EXPECT_GT(currentGuardValues[1], -1e-6);
  }

  // Test 3: Event times
  scalar_array_t eventTestTimes = {0.126835459, 1.73439091, 2.25798967, 3.86554513, 4.38914388};
  EXPECT_EQ(eventTestTimes.size(), eventsPastTheEndIndeces.size());
  for (int i = 0; i < eventsPastTheEndIndeces.size(); i++) {
    if (i < eventTestTimes.size()) {
      EXPECT_LT(std::fabs(timeTrajectory[eventsPastTheEndIndeces[i] - 1] - eventTestTimes[i]), 1e-8);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
