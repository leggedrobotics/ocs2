#include <gtest/gtest.h>

#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/StateBasedLinearController.h>
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
TEST(testStateRollOut_SLQ, DISABLED_BouncingMassTest) {
  using scalar_t = ocs2::scalar_t;
  using vector_t = ocs2::vector_t;
  using matrix_t = ocs2::matrix_t;
  using scalar_array_t = ocs2::scalar_array_t;
  using vector_array_t = ocs2::vector_array_t;
  using matrix_array_t = ocs2::matrix_array_t;

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
  slqSettings.ddpSettings_.maxNumStepsPerSecond_ = 10000;
  slqSettings.ddpSettings_.useFeedbackPolicy_ = true;
  slqSettings.ddpSettings_.debugPrintRollout_ = false;

  ocs2::Rollout_Settings rolloutSettings;
  rolloutSettings.absTolODE_ = 1e-10;
  rolloutSettings.relTolODE_ = 1e-7;
  rolloutSettings.maxNumStepsPerSecond_ = 10000;
  rolloutSettings.maxSingleEventIterations_ = 5;
  rolloutSettings.useTrajectorySpreadingController_ = true;

  // Parameters
  const scalar_t startTime = 0.0;
  const scalar_t finalTime = 2.5;
  const vector_t x0 = Eigen::Matrix<scalar_t, 3, 1>(0.7, 0.5, 0);
  const vector_t u0 = vector_t::Zero(INPUT_DIM);
  bool outputSolution = false;

  // Generation of Reference Trajectory
  const scalar_array_t trajTimes{0, 0.2, 0.8, 1.0, 1.2, 1.8, 2.0};

  vector_t state0(STATE_DIM);  //	Intial and final state
  state0 << 0.5, 0, 0;
  vector_t state1(STATE_DIM);  //	Hard impact
  state1 << 0, -5, 0;
  vector_t state2(STATE_DIM);  // 	Soft impact
  state2 << 0, -1, 0;

  const scalar_t delta = 0.5;
  const vector_array_t trajStates{state0, state1, state2, state2, state1, state2, state0};
  OverallReference reference(trajTimes, trajStates);
  reference.extendref(delta);

  // Dynamics, Constraints and derivative classes
  systemDynamics systemModel;
  systemDerivative systemDerivatives;
  systemConstraint systemConstraints;

  // Cost Function
  matrix_t Q(STATE_DIM, STATE_DIM);
  Q << 50.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0;

  matrix_t R(INPUT_DIM, INPUT_DIM);
  R << 1.0;
  matrix_t P(STATE_DIM, STATE_DIM);
  P << 56.63, 7.07, 0.0, 7.07, 8.01, 0.0, 0.0, 0.0, 0.0;

  vector_t xNom = state0;
  vector_t uNom(INPUT_DIM);
  uNom << 0;
  vector_t xFin = state0;
  systemCost systemCost(reference, Q, R, P, xNom, uNom, xFin, finalTime);

  // Rollout Class
  ocs2::StateTriggeredRollout stateTriggeredRollout(systemModel, rolloutSettings);

  // Operating points and PartitioningTimes
  scalar_array_t partitioningTimes;
  partitioningTimes.push_back(startTime);
  partitioningTimes.push_back(finalTime);

  // Initial Controller
  matrix_t controllerGain(INPUT_DIM, STATE_DIM);
  controllerGain << 25, 10, 0;
  vector_t controllerBias;

  matrix_array_t controllerGainArray;
  vector_array_t controllerBiasArray;
  scalar_array_t timeStampArray;

  const scalar_t controllerDeltaTime = 1e-3;  // Time step for controller time array
  const scalar_t eps = ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon();
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

  ocs2::LinearController Control(STATE_DIM, INPUT_DIM, timeStampArray, controllerBiasArray, controllerGainArray);
  std::vector<ocs2::ControllerBase*> controllerPtrArray = {&Control};

  ocs2::OperatingPoints operatingTrajectories(x0, u0);
  // SLQ
  ocs2::SLQ slq(STATE_DIM, INPUT_DIM, &stateTriggeredRollout, &systemDerivatives, &systemConstraints, &systemCost, &operatingTrajectories,
                slqSettings);
  slq.run(startTime, x0, finalTime, partitioningTimes, controllerPtrArray);
  auto solutionST = slq.primalSolution(finalTime);

  for (int i = 0; i < solutionST.stateTrajectory_.size(); i++) {
    // Test 1: No penetration of Guard Surfaces
    EXPECT_GT(solutionST.stateTrajectory_[i][0], -slqSettings.ddpSettings_.absTolODE_);
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
  const scalar_t expectedCost = 7.188299;
  EXPECT_LT(std::fabs(performanceIndeces.totalCost - expectedCost), 100 * slqSettings.ddpSettings_.minRelCost_);
}
