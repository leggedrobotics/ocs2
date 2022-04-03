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
#include <iostream>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_robotic_assets/package_path.h>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

class MobileManipulatorIntegrationTest : public testing::Test {
 protected:
  using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
  using quaternion_t = Eigen::Quaternion<scalar_t, Eigen::DontAlign>;

  MobileManipulatorIntegrationTest() {
    const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/mabi_mobile/task.info";
    const std::string libFolder = ocs2::mobile_manipulator::getPath() + "/auto_generated/mabi_mobile";
    const std::string urdfFile = ocs2::robotic_assets::getPath() + "/resources/mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf";
    mobileManipulatorInterfacePtr.reset(new MobileManipulatorInterface(taskFile, libFolder, urdfFile));
    // obtain robot model info
    modelInfo = mobileManipulatorInterfacePtr->getManipulatorModelInfo();

    // initialize reference
    const vector_t goalState = (vector_t(7) << goalPosition, goalOrientation.coeffs()).finished();
    TargetTrajectories targetTrajectories({initTime}, {goalState}, {vector_t::Zero(modelInfo.inputDim)});
    mobileManipulatorInterfacePtr->getReferenceManagerPtr()->setTargetTrajectories(std::move(targetTrajectories));

    // initialize kinematics
    const std::string modelName = "end_effector_kinematics_dummytest";
    MobileManipulatorPinocchioMappingCppAd pinocchioMapping(modelInfo);
    const auto& pinocchioInterface = mobileManipulatorInterfacePtr->getPinocchioInterface();
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(pinocchioInterface, pinocchioMapping, {modelInfo.eeFrame},
                                                                  modelInfo.stateDim, modelInfo.inputDim, modelName));
  }

  std::unique_ptr<GaussNewtonDDP_MPC> getMpc() {
    auto& interface = *mobileManipulatorInterfacePtr;
    std::unique_ptr<GaussNewtonDDP_MPC> mpcPtr(new GaussNewtonDDP_MPC(interface.mpcSettings(), interface.ddpSettings(),
                                                                      interface.getRollout(), interface.getOptimalControlProblem(),
                                                                      interface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(mobileManipulatorInterfacePtr->getReferenceManagerPtr());
    return mpcPtr;
  }

  void verifyTrackingQuality(const vector_t& state) const {
    const vector3_t eePositionError = eeKinematicsPtr->getPosition(state).front() - goalPosition;
    const vector3_t eeOrientationError = eeKinematicsPtr->getOrientationError(state, {goalOrientation}).front();
    std::cerr << "eePositionError: " << eePositionError.transpose() << '\n';
    std::cerr << "eeOrientationError: " << eeOrientationError.transpose() << '\n';
    // check that goal position is reached
    EXPECT_NEAR(eePositionError.x(), 0.0, tolerance);
    EXPECT_NEAR(eePositionError.y(), 0.0, tolerance);
    EXPECT_NEAR(eePositionError.z(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.x(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.y(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.z(), 0.0, tolerance);
  }

  static constexpr scalar_t tolerance = 1e-2;
  static constexpr scalar_t f_mpc = 10.0;
  static constexpr scalar_t initTime = 1234.5;  // start from a random time
  static constexpr scalar_t finalTime = initTime + 10.0;

  const vector3_t goalPosition = vector3_t(-0.5, -0.8, 0.6);
  const quaternion_t goalOrientation = quaternion_t(0.33, 0.0, 0.0, 0.95);

  ManipulatorModelInfo modelInfo;
  std::unique_ptr<MobileManipulatorInterface> mobileManipulatorInterfacePtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsPtr;
};

constexpr scalar_t MobileManipulatorIntegrationTest::tolerance;
constexpr scalar_t MobileManipulatorIntegrationTest::f_mpc;
constexpr scalar_t MobileManipulatorIntegrationTest::initTime;
constexpr scalar_t MobileManipulatorIntegrationTest::finalTime;

TEST_F(MobileManipulatorIntegrationTest, synchronousTracking) {
  auto mpcPtr = getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  SystemObservation observation;
  observation.time = initTime;
  observation.state = mobileManipulatorInterfacePtr->getInitialState();
  observation.input.setZero(modelInfo.inputDim);
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
      mpcInterface.evaluatePolicy(time, vector_t::Zero(modelInfo.stateDim), optimalState, optimalInput, mode);

      // use optimal state for the next observation:
      observation.time = time;
      observation.state = optimalState;
      observation.input.setZero(modelInfo.inputDim);
      mpcInterface.setCurrentObservation(observation);
    }
  }

  verifyTrackingQuality(observation.state);
}

TEST_F(MobileManipulatorIntegrationTest, asynchronousTracking) {
  auto mpcPtr = getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  const scalar_t f_mrt = 100.0;  // Hz

  // Set initial observation
  SystemObservation observation;
  observation.time = initTime;
  observation.state = mobileManipulatorInterfacePtr->getInitialState();
  observation.input.setZero(modelInfo.inputDim);

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
          mpcInterface.evaluatePolicy(observation.time, vector_t::Zero(modelInfo.stateDim), observation.state, observation.input,
                                      observation.mode);

          // use optimal state for the next observation:
          mpcInterface.setCurrentObservation(observation);
        },
        f_mrt);
  }

  mpcRunning = false;
  if (mpcThread.joinable()) {
    mpcThread.join();
  }

  verifyTrackingQuality(observation.state);
}
