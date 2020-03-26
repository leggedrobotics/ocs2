#include <gtest/gtest.h>
#include <iostream>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/StateBasedLinearController.h>
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
TEST(testStateRollOut_SLQ, BouncingMassTest) {
  using DIMENSIONS = ocs2::Dimensions<3, 1>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using controller_t = ocs2::ControllerBase<STATE_DIM, INPUT_DIM>;
  using linear_controller_t = ocs2::LinearController<STATE_DIM, INPUT_DIM>;

  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;

  using controller_ptr_array_t = std::vector<controller_t*>;

  ocs2::SLQ_Settings slqSettings;
  slqSettings.useNominalTimeForBackwardPass_ = true;
  slqSettings.ddpSettings_.displayInfo_ = false;
  slqSettings.ddpSettings_.displayShortSummary_ = true;
  slqSettings.ddpSettings_.maxNumIterations_ = 30;
  slqSettings.ddpSettings_.minRelCost_ = 1e-4;
  slqSettings.ddpSettings_.nThreads_ = 1;
  slqSettings.ddpSettings_.checkNumericalStability_ = true;
  slqSettings.ddpSettings_.absTolODE_ = 1e-10;
  slqSettings.ddpSettings_.relTolODE_ = 1e-7;
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 1e7;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

  ocs2::Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 1e7;
  rolloutSettings.maxSingleEventIterations_ = 5;
  rolloutSettings.useTrajectorySpreadingController_ = true;

  // Parameters
  const double startTime = 0.0;
  const double finalTime = 2.5;
  const state_vector_t x0 = {0.7, 0.5, 0};
  bool outputSolution = false;

  // Generation of Reference Trajectory
  const scalar_array_t trajTimes{0, 0.2, 0.8, 1.0, 1.2, 1.8, 2.0};

  state_vector_t state0;  //	Intial and final state
  state0 << 0.5, 0, 0;
  state_vector_t state1;  //	Hard impact
  state1 << 0, -5, 0;
  state_vector_t state2;  // 	Soft impact
  state2 << 0, -1, 0;

  const scalar_t delta = 0.5;
  const state_vector_array_t trajStates{state0, state1, state2, state2, state1, state2, state0};
  OverallReference reference(trajTimes, trajStates);
  reference.extendref(delta);

  // Dynamics, Constraints and derivative classes
  systemDynamics systemModel;
  systemDerivative systemDerivatives;
  systemConstraint systemConstraints;

  // Cost Function
  state_matrix_t Q;
  Q << 50.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0;

  input_matrix_t R(1);
  state_matrix_t P;
  P << 56.63, 7.07, 0.0, 7.07, 8.01, 0.0, 0.0, 0.0, 0.0;

  state_vector_t xNom = state0;
  input_vector_t uNom(0);
  state_vector_t xFin = state0;
  systemCost systemCost(reference, Q, R, P, xNom, uNom, xFin, finalTime);

  // Rollout Class
  ocs2::StateTriggeredRollout<STATE_DIM, INPUT_DIM> stateTriggeredRollout(systemModel, rolloutSettings);

  // Operating points and PartitioningTimes
  scalar_array_t partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime);

  // Initial Controller
  const input_state_matrix_t controllerGain = {25, 10, 0};
  input_vector_t controllerBias;

  input_state_matrix_array_t controllerGainArray;
  input_vector_array_t controllerBiasArray;
  scalar_array_t timeStampArray;

  const scalar_t controllerDeltaTime = 1e-3;  // Time step for controller time array
  const scalar_t eps = ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon();
  scalar_array_t controlTimes = trajTimes;
  controlTimes.push_back(finalTime);

  for (int i = 0; i < controlTimes.size() - 1; i++) {
    scalar_t timeSteps = (controlTimes[i + 1] + eps - controlTimes[i]) / controllerDeltaTime;
    scalar_t timeStamp = 0;
    state_vector_t refState;

    for (int j = 0; j <= timeSteps; j++) {
      if (j == 0 && i < controlTimes.size() - 2) {
        timeStamp = controlTimes[i] + eps;
      } else {
        timeStamp = controlTimes[i] + j * controllerDeltaTime;
      }

      state_vector_t refState = reference.getState(timeStamp);
      input_vector_t refInput = reference.getInput(timeStamp);
      controllerBias = controllerGain * refState + refInput;

      timeStampArray.push_back(timeStamp);
      controllerGainArray.push_back(-controllerGain);
      controllerBiasArray.push_back(controllerBias);
    }
  }

  linear_controller_t Control(timeStampArray, controllerBiasArray, controllerGainArray);
  controller_ptr_array_t controllerPtrArray = {&Control};

  systemOperatingTrajectories operatingTrajectories;
  // SLQ
  ocs2::SLQ<STATE_DIM, INPUT_DIM> slq(&stateTriggeredRollout, &systemDerivatives, &systemConstraints, &systemCost, &operatingTrajectories,
                                      slqSettings);
  slq.run(startTime, x0, finalTime, partitioningTimes, controllerPtrArray);
  ocs2::SLQ<STATE_DIM, INPUT_DIM>::primal_solution_t solutionST = slq.primalSolution(finalTime);

  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    // Test 1: No penetration of Guard Surfaces
    EXPECT_GT(solutionST.stateTrajectory_[i][0], -slqSettings.ddpSettings_.absTolODE_);
    // Display output
    // format: idx;time;x[0];xref[0];x[1];xref[1];u;uref
    if (outputSolution) {
      int idx = solutionST.stateTrajectory_[i][2];

      input_vector_t uRef = reference.getInput(solutionST.timeTrajectory_[i]);
      state_vector_t xRef = reference.getState(idx, solutionST.timeTrajectory_[i]);

      std::cerr << i << ";" << idx << ";";
      std::cerr << std::setprecision(25) << solutionST.timeTrajectory_[i];
      std::cerr << std::setprecision(6) << ";" << solutionST.stateTrajectory_[i][0] << ";" << xRef[0] << ";";
      std::cerr << solutionST.stateTrajectory_[i][1] << ";" << xRef[1] << ";";
      std::cerr << solutionST.inputTrajectory_[i][0] << ";" << uRef[0] << std::endl;
    }
  }

  // Test 2: Check of cost function
  auto performanceIndeces = slq.getPerformanceIndeces();
  const scalar_t expectedCost = 7.188299;
  EXPECT_LT(std::fabs(performanceIndeces.totalCost - expectedCost), 10 * slqSettings.ddpSettings_.minRelCost_);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
