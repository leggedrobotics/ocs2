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

#include <cmath>

#include <gtest/gtest.h>

#include <ocs2_double_integrator_example/DoubleIntegratorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

using namespace ocs2;
using namespace double_integrator;

class DoubleIntegratorIntegrationTest : public testing::Test {
 protected:
  DoubleIntegratorIntegrationTest() {
    const bool verbose = false;
    doubleIntegratorInterfacePtr.reset(new DoubleIntegratorInterface("mpc", verbose));

    initState = doubleIntegratorInterfacePtr->getInitialState();
    goalState = doubleIntegratorInterfacePtr->getInitialTarget();

    // initialize reference
    targetTrajectories.timeTrajectory.push_back(initTime);
    targetTrajectories.timeTrajectory.push_back(initTime + 1.0);
    targetTrajectories.stateTrajectory.push_back(initState);
    targetTrajectories.stateTrajectory.push_back(goalState);
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  }

  const scalar_t tolerance = 2e-2;
  const scalar_t f_mpc = 10.0;
  const scalar_t mpcIncrement = 1.0 / f_mpc;
  const scalar_t initTime = 1234.5;  // start from a random time
  const scalar_t finalTime = initTime + 5.0;

  vector_t initState;
  vector_t goalState;
  std::unique_ptr<DoubleIntegratorInterface> doubleIntegratorInterfacePtr;
  TargetTrajectories targetTrajectories;
};

TEST_F(DoubleIntegratorIntegrationTest, synchronousTracking) {
  auto mpcPtr = doubleIntegratorInterfacePtr->getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.setTargetTrajectories(targetTrajectories);

  SystemObservation observation;
  observation.time = initTime;
  observation.state = initState;
  observation.input.setZero(INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  // run MPC for N iterations
  auto time = initTime;
  const auto N = static_cast<size_t>(f_mpc * (finalTime - initTime));
  for (size_t i = 0; i < N; i++) {
    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_mpc;

    if (mpcInterface.initialPolicyReceived()) {
      size_t mode;
      vector_t optimalState, optimalInput;

      mpcInterface.updatePolicy();
      mpcInterface.evaluatePolicy(time, vector_t::Zero(STATE_DIM), optimalState, optimalInput, mode);

      // use optimal state for the next observation:
      observation.time = time;
      observation.state = optimalState;
      observation.input.setZero(INPUT_DIM);
      mpcInterface.setCurrentObservation(observation);
    }
  }

  ASSERT_NEAR(observation.state(0), goalState(0), tolerance);
}

TEST_F(DoubleIntegratorIntegrationTest, coldStartMPC) {
  auto mpcPtr = doubleIntegratorInterfacePtr->getMpc(false);
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.setTargetTrajectories(targetTrajectories);

  SystemObservation observation;
  observation.time = initTime;
  observation.state = initState;
  observation.input.setZero(INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  // run MPC for N iterations
  auto time = initTime;
  const auto N = static_cast<size_t>(f_mpc * (finalTime - initTime));
  for (size_t i = 0; i < N; i++) {
    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_mpc;

    if (mpcInterface.initialPolicyReceived()) {
      size_t mode;
      vector_t optimalState, optimalInput;

      mpcInterface.updatePolicy();
      mpcInterface.evaluatePolicy(time, vector_t::Zero(STATE_DIM), optimalState, optimalInput, mode);

      // use optimal state for the next observation:
      observation.time = time;
      observation.state = optimalState;
      observation.input.setZero(INPUT_DIM);
      mpcInterface.setCurrentObservation(observation);
    }
  }

  ASSERT_NEAR(observation.state(0), goalState(0), tolerance);
}

TEST_F(DoubleIntegratorIntegrationTest, asynchronousTracking) {
  auto mpcPtr = doubleIntegratorInterfacePtr->getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.setTargetTrajectories(targetTrajectories);

  const scalar_t f_mrt = 100;
  const scalar_t mrtTimeIncrement = 1.0 / f_mrt;

  scalar_t time = initTime;
  size_t mode;
  vector_t optimalState = initState;
  vector_t optimalInput;

  // run MRT in a thread
  std::mutex timeStateMutex;
  std::atomic_bool trackerRunning{true};
  auto tracker = [&]() {
    while (trackerRunning) {
      {
        std::lock_guard<std::mutex> lock(timeStateMutex);
        time += mrtTimeIncrement;
        if (mpcInterface.initialPolicyReceived()) {
          mpcInterface.updatePolicy();
          mpcInterface.evaluatePolicy(time, vector_t::Zero(STATE_DIM), optimalState, optimalInput, mode);
        }
        if (std::abs(time - finalTime) < 0.005) {
          ASSERT_NEAR(optimalState(0), goalState(0), tolerance);
        }
      }
      usleep(uint(mrtTimeIncrement * 1e6));
    }
  };
  std::thread trackerThread(tracker);

  try {
    // run MPC for N iterations
    SystemObservation observation;
    const auto N = static_cast<size_t>(f_mpc * (finalTime - initTime));
    for (size_t i = 0; i < N; i++) {
      {
        std::lock_guard<std::mutex> lock(timeStateMutex);
        // use optimal state for the next observation:
        observation.time = time;
        observation.state = optimalState;
        observation.input.setZero(INPUT_DIM);
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
