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
#include <algorithm>

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

// Aliases
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using quaternion_t = Eigen::Quaternion<scalar_t, Eigen::DontAlign>;

/**
 * @brief Test fixture for checking end-effector tracking using kinematic formulation for MPC.
 * 
 * @tparam A tuple containing the task file, library folder, URDF file, goal position and goal orientation.
 */
class DummyMobileManipulatorParametersTests
    : public testing::TestWithParam <std::tuple<std::string, std::string, std::string, vector3_t, quaternion_t>> {
protected:
  
  // Constants
  static constexpr scalar_t tolerance = 1e-2;
  static constexpr scalar_t f_mpc = 10.0;
  static constexpr scalar_t initTime = 1234.5; // start from a random time
  static constexpr scalar_t finalTime = initTime + 10.0;

  // Resolve test fixture parameters
  const std::string getTaskFile() const {
    return ocs2::mobile_manipulator::getPath() + "/config/" + std::get<0>(GetParam());
  }
  const std::string getLibFolder() const {
    return ocs2::mobile_manipulator::getPath() + "/auto_generated/" + std::get<1>(GetParam());
  }
  const std::string getUrdfFile() const {
    return ocs2::robotic_assets::getPath() + "/resources/mobile_manipulator/" + std::get<2>(GetParam());
  }
  const vector3_t getGoalPosition() const {
    return std::get<3>(GetParam());
  }
  const quaternion_t getGoalOrientation() const {
    return std::get<4>(GetParam());
  }

  /**
   * @note: We separate `initialize()` and `getMpc()` since one can obtain
   * multiple MPC instances from same interface.
   */
  bool initialize(const std::string& taskFile, const std::string& libFolder, const std::string& urdfFile) {
    // create mpc interface
    mobileManipulatorInterfacePtr.reset(
        new MobileManipulatorInterface(taskFile, libFolder, urdfFile));
    // obtain robot model info
    modelInfo = mobileManipulatorInterfacePtr->getManipulatorModelInfo();

    // initialize reference
    const vector_t goalState = (vector_t(7) << getGoalPosition(), getGoalOrientation().coeffs()).finished();
    TargetTrajectories targetTrajectories({initTime}, {goalState}, {vector_t::Zero(modelInfo.inputDim)});
    mobileManipulatorInterfacePtr->getReferenceManagerPtr()->setTargetTrajectories(std::move(targetTrajectories));

    // initialize kinematics
    const std::string modelName = "end_effector_kinematics_dummytest";
    MobileManipulatorPinocchioMappingCppAd pinocchioMapping(modelInfo);
    const auto& pinocchioInterface = mobileManipulatorInterfacePtr->getPinocchioInterface();
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(
        pinocchioInterface, pinocchioMapping, {modelInfo.eeFrame},
        modelInfo.stateDim, modelInfo.inputDim, modelName));
    return true;
  }

  std::unique_ptr<GaussNewtonDDP_MPC> getMpc() {
    auto& interface = *mobileManipulatorInterfacePtr;
    std::unique_ptr<GaussNewtonDDP_MPC> mpcPtr(new GaussNewtonDDP_MPC(
        interface.mpcSettings(), interface.ddpSettings(),
        interface.getRollout(), interface.getOptimalControlProblem(),
        interface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(
        mobileManipulatorInterfacePtr->getReferenceManagerPtr());
    return mpcPtr;
  }

  void verifyTrackingQuality(const vector_t& state) const {
    const vector3_t eePositionError = eeKinematicsPtr->getPosition(state).front() - getGoalPosition();
    const vector3_t eeOrientationError = eeKinematicsPtr->getOrientationError(state, {getGoalOrientation()}).front();
    // test report
    std::cerr << "[SUMMARY]: ------------------------------------------------------\n";
    std::cerr << getTestName();
    std::cerr << "\teePositionError: " << eePositionError.transpose() << '\n';
    std::cerr << "\teeOrientationError: " << eeOrientationError.transpose() << '\n';
    std::cerr << "-----------------------------------------------------------------\n";
    // check that goal position is reached
    EXPECT_NEAR(eePositionError.x(), 0.0, tolerance);
    EXPECT_NEAR(eePositionError.y(), 0.0, tolerance);
    EXPECT_NEAR(eePositionError.z(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.x(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.y(), 0.0, tolerance);
    EXPECT_NEAR(eeOrientationError.z(), 0.0, tolerance);
  }

  const std::string getTestName() const {
    std::string testName;
    testName += "DummyMobileManipulatorParametersTests Test {";
    testName += "\n\tTask: " + getTaskFile();
    testName += "\n\tURDF: " + getUrdfFile();
    testName += "\n}\n";
    return testName;
  }

  ManipulatorModelInfo modelInfo;
  std::unique_ptr<MobileManipulatorInterface> mobileManipulatorInterfacePtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsPtr;
};

// expose constants globally
constexpr scalar_t DummyMobileManipulatorParametersTests::tolerance;
constexpr scalar_t DummyMobileManipulatorParametersTests::f_mpc;
constexpr scalar_t DummyMobileManipulatorParametersTests::initTime;
constexpr scalar_t DummyMobileManipulatorParametersTests::finalTime;

TEST_P(DummyMobileManipulatorParametersTests, synchronousTracking) {
  // Obtain mpc
  initialize(getTaskFile(), getLibFolder(), getUrdfFile());
  auto mpcPtr = getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  // Set initial observation
  SystemObservation observation;
  observation.time = initTime;
  observation.state = mobileManipulatorInterfacePtr->getInitialState();
  observation.input.setZero(modelInfo.inputDim);
  mpcInterface.setCurrentObservation(observation);

  // Run MPC for N iterations
  auto time = initTime;
  while (time < finalTime) {
    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_mpc;

    if (mpcInterface.initialPolicyReceived()) {
      size_t mode;
      vector_t optimalState, optimalInput;

      mpcInterface.updatePolicy();
      mpcInterface.evaluatePolicy(time, vector_t::Zero(modelInfo.stateDim),
                                  optimalState, optimalInput, mode);

      // use optimal state for the next observation:
      observation.time = time;
      observation.state = optimalState;
      observation.input.setZero(modelInfo.inputDim);
      mpcInterface.setCurrentObservation(observation);
    }
  }

  verifyTrackingQuality(observation.state);
}

TEST_P(DummyMobileManipulatorParametersTests, asynchronousTracking) {
  // Obtain mpc
  initialize(getTaskFile(), getLibFolder(), getUrdfFile());
  auto mpcPtr = getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);

  const scalar_t f_mrt = 100.0; // Hz

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
          mpcInterface.evaluatePolicy(
              observation.time, vector_t::Zero(modelInfo.stateDim),
              observation.state, observation.input, observation.mode);

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(
    DummyMobileManipulatorTests, DummyMobileManipulatorParametersTests,
    testing::Values(
        // franka panda: 7-Dof arm
        std::make_tuple("franka/task.info", "franka", "franka/urdf/panda.urdf",
                        vector3_t(0.4, 0.1, 0.5), quaternion_t(0.33, 0.0, 0.0, 0.95)),
        // kinova jaco2: 6-Dof arm
        std::make_tuple("kinova/task_j2n6.info", "kinova/j2n6",
                        "kinova/urdf/j2n6s300.urdf", vector3_t(0.2, 0.2, 0.6), 
                        quaternion_t(0.33, 0.0, 0.0, 0.95)),
        // kinova jaco2: 7-Dof arm
        std::make_tuple("kinova/task_j2n7.info", "kinova/j2n7",
                        "kinova/urdf/j2n7s300.urdf", vector3_t(0.2, 0.2, 0.6),
                        quaternion_t(0.33, 0.0, 0.0, 0.95)),
        // mabi-mobile: SE(2) + 6-Dof arm
        std::make_tuple("mabi_mobile/task.info", "mabi_mobile",
                        "mabi_mobile/urdf/mabi_mobile.urdf", 
                        vector3_t(-0.5, -0.8, 0.6), quaternion_t(0.33, 0.0, 0.0, 0.95)),
        // OSRF PR2: SE(2) + 7-Dof arm
        std::make_tuple("pr2/task.info", "pr2", "pr2/urdf/pr2.urdf",
                        vector3_t(-0.5, -0.8, 0.6), quaternion_t(0.33, 0.0, 0.0, 0.95)),
        // ridgeback with ur5: SE(2) + 6-Dof arm
        std::make_tuple("ridgeback_ur5/task.info", "ridgeback_ur5",
                        "ridgeback_ur5/urdf/ridgeback_ur5.urdf", 
                        vector3_t(-0.5, -0.8, 0.6), quaternion_t(0.33, 0.0, 0.0, 0.95))
      ),
    [](const testing::TestParamInfo<DummyMobileManipulatorParametersTests::ParamType>& info) {
      /* returns test name for gtest summary */
      std::string robotName = std::get<1>(info.param);
      std::replace(robotName.begin(), robotName.end(), '/', '_' );
      return "Task__" + robotName;
    });
