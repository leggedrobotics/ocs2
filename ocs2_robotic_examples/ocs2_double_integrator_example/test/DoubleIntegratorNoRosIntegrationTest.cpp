#include <gtest/gtest.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_MRT_Interface.h>
#include <ocs2_double_integrator_example/DoubleIntegratorInterface.h>
#include <cmath>

using namespace ocs2;
using namespace double_integrator;

TEST(DoubleIntegratorIntegrationTest, synchronousTracking) {
  std::string taskFileFolderName = "mpc";
  DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolderName);
  auto mpcPtr = doubleIntegratorInterface.getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  double time = 1234.5;  // start from a random time

  // initialize observation:
  SystemObservation observation;
  observation.state() = doubleIntegratorInterface.getInitialState();
  observation.time() = time;

  mpcInterface.setCurrentObservation(observation);

  // initialize reference:
  CostDesiredTrajectories costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time + 1);
  vector_t goalState = doubleIntegratorInterface.getXFinal();
  costDesiredTrajectories.desiredStateTrajectory().push_back(observation.state());
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  vector_t desiredInput = vector_t::Zero(INPUT_DIM_);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface.setTargetTrajectories(costDesiredTrajectories);

  double f_control = 10;
  double T = 5;

  // run MPC for N iterations
  int N = static_cast<int>(f_control * T);
  for (int i = 0; i < N; i++) {
    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_control;

    if (mpcInterface.initialPolicyReceived()) {
      vector_t optimalState;
      vector_t optimalInput;
      size_t subsystem;

      mpcInterface.updatePolicy();
      mpcInterface.evaluatePolicy(time, vector_t::Zero(STATE_DIM_), optimalState, optimalInput, subsystem);

      // use optimal state for the next observation:
      observation.state() = optimalState;
      observation.time() = time;
      mpcInterface.setCurrentObservation(observation);
    }
  }

  ASSERT_NEAR(observation.state()[0], goalState[0], 2e-2);
}

TEST(DoubleIntegratorIntegrationTest, asynchronousTracking) {
  // task file
  std::string taskFileFolderName = "mpc";

  DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolderName);
  auto mpcPtr = doubleIntegratorInterface.getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  double time = 1234.5;  // start from a random time

  vector_t initialState;
  initialState = doubleIntegratorInterface.getInitialState();

  // initialize reference:
  CostDesiredTrajectories costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time);
  costDesiredTrajectories.desiredTimeTrajectory().push_back(time + 1);
  vector_t goalState = doubleIntegratorInterface.getXFinal();
  costDesiredTrajectories.desiredStateTrajectory().push_back(initialState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(goalState);
  vector_t desiredInput = vector_t::Zero(INPUT_DIM_);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  costDesiredTrajectories.desiredInputTrajectory().push_back(desiredInput);
  mpcInterface.setTargetTrajectories(costDesiredTrajectories);

  double f_mpc = 10;
  double mpcIncrement = 1.0 / f_mpc;
  double f_tracking = 100;
  double trackingIncrement = 1.0 / f_tracking;
  double T = 5;

  SystemObservation observation;
  vector_t optimalState = initialState;
  vector_t optimalInput;
  size_t subsystem;
  std::atomic_bool trackerRunning(true);

  std::mutex timeStateMutex;

  auto tracker = [&]() {
    while (trackerRunning) {
      {
        std::lock_guard<std::mutex> lock(timeStateMutex);
        time += trackingIncrement;
        if (mpcInterface.initialPolicyReceived()) {
          mpcInterface.updatePolicy();
          mpcInterface.evaluatePolicy(time, vector_t::Zero(STATE_DIM_), optimalState, optimalInput, subsystem);
        }
        if (std::abs(time - T) < 0.005) {
          ASSERT_NEAR(optimalState[0], goalState[0], 2e-2);
        }
      }
      usleep(uint(trackingIncrement * 1e6));
    }
  };

  std::thread trackerThread(tracker);

  try {
    // run MPC for N iterations
    int N = int(f_mpc * T);
    for (int i = 0; i < N; i++) {
      {
        std::lock_guard<std::mutex> lock(timeStateMutex);
        // use optimal state for the next observation:
        observation.state() = optimalState;
        observation.time() = time;
      }
      mpcInterface.setCurrentObservation(observation);
      mpcInterface.advanceMpc();

      usleep(uint(mpcIncrement * 1e6));
    }
  } catch (const std::exception& e) {
    std::cerr << "EXCEPTION " << e.what() << std::endl;
    EXPECT_TRUE(false);
  }
  trackerRunning = false;
  trackerThread.join();
}
