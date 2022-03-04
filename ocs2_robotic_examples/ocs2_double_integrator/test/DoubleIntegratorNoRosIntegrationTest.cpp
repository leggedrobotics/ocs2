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

#include <ocs2_double_integrator/DoubleIntegratorInterface.h>
#include <ocs2_double_integrator/package_path.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

using namespace ocs2;
using namespace double_integrator;

class DoubleIntegratorIntegrationTest : public testing::Test {
 protected:
  DoubleIntegratorIntegrationTest() {
    const bool verbose = false;
    const std::string taskFile = ocs2::double_integrator::getPath() + "/config/mpc/task.info";
    const std::string libFolder = ocs2::double_integrator::getPath() + "/auto_generated";
    doubleIntegratorInterfacePtr.reset(new DoubleIntegratorInterface(taskFile, libFolder, verbose));

    initState = doubleIntegratorInterfacePtr->getInitialState();
    goalState = doubleIntegratorInterfacePtr->getInitialTarget();

    // initialize reference
    TargetTrajectories targetTrajectories({initTime}, {goalState}, {vector_t::Zero(INPUT_DIM)});
    doubleIntegratorInterfacePtr->getReferenceManagerPtr()->setTargetTrajectories(std::move(targetTrajectories));
  }

  std::unique_ptr<GaussNewtonDDP_MPC> getMpc(bool warmStart) {
    auto& interface = *doubleIntegratorInterfacePtr;
    auto mpcSettings = interface.mpcSettings();
    auto ddpSettings = interface.ddpSettings();
    if (!warmStart) {
      mpcSettings.coldStart_ = true;
      ddpSettings.maxNumIterations_ = 5;
    }

    std::unique_ptr<GaussNewtonDDP_MPC> mpcPtr(new GaussNewtonDDP_MPC(mpcSettings, ddpSettings, interface.getRollout(),
                                                                      interface.getOptimalControlProblem(), interface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());

    return mpcPtr;
  }

  const scalar_t tolerance = 2e-2;
  const scalar_t f_mpc = 10.0;
  const scalar_t initTime = 1234.5;  // start from a random time
  const scalar_t finalTime = initTime + 5.0;

  vector_t initState;
  vector_t goalState;
  std::unique_ptr<DoubleIntegratorInterface> doubleIntegratorInterfacePtr;
};

TEST_F(DoubleIntegratorIntegrationTest, synchronousTracking) {
  auto mpcPtr = getMpc(true);
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  SystemObservation observation;
  observation.time = initTime;
  observation.state = initState;
  observation.input.setZero(INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  // run MPC for N iterations
  auto time = initTime;
  while (time < finalTime) {
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
  auto mpcPtr = getMpc(false);
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  SystemObservation observation;
  observation.time = initTime;
  observation.state = initState;
  observation.input.setZero(INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  // run MPC for N iterations
  auto time = initTime;
  while (time < finalTime) {
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
  auto mpcPtr = getMpc(true);
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  const scalar_t f_mrt = 100;

  SystemObservation observation;
  observation.time = initTime;
  observation.state = initState;
  observation.input.setZero(INPUT_DIM);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) {
    mpcInterface.advanceMpc();
  }

  // Run MPC in a thread
  std::atomic_bool mpcRunning{true};
  auto mpcThread = std::thread([&]() {
    while (mpcRunning) {
      try {
        ocs2::executeAndSleep([&]() { mpcInterface.advanceMpc(); }, f_mpc);
      } catch (const std::exception& e) {
        mpcRunning = false;
        std::cerr << "EXCEPTION " << e.what() << std::endl;
        EXPECT_TRUE(false);
      }
    }
  });

  // run MRT
  while (observation.time < finalTime) {
    ocs2::executeAndSleep(
        [&]() {
          observation.time += 1.0 / f_mrt;

          // Evaluate the policy
          mpcInterface.updatePolicy();
          mpcInterface.evaluatePolicy(observation.time, vector_t::Zero(STATE_DIM), observation.state, observation.input, observation.mode);

          // use optimal state for the next observation:
          mpcInterface.setCurrentObservation(observation);
        },
        f_mrt);
  }

  mpcRunning = false;
  if (mpcThread.joinable()) {
    mpcThread.join();
  }

  ASSERT_NEAR(observation.state(0), goalState(0), tolerance);
}
