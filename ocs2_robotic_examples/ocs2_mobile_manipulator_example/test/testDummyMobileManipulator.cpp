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
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <gtest/gtest.h>

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorPinocchioMapping.h>

using namespace ocs2;
using namespace mobile_manipulator;

class MobileManipulatorIntegrationTest : public testing::Test {
protected:
  MobileManipulatorIntegrationTest() {
    mobileManipulatorInterfacePtr.reset(new MobileManipulatorInterface("mpc"));

    // urdf file path
    const std::string packageDir = "/home/mayank/git_devel/ocs2_dev/ocs2_robotic_examples/ocs2_mobile_manipulator_example";
    const std::string urdfPath = packageDir + "/urdf/mobile_manipulator.urdf";
//    const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

    // initialize pinocchio
    pinocchioInterface.reset(new PinocchioInterface(MobileManipulatorInterface::buildPinocchioInterface(urdfPath)));
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematics(*pinocchioInterface, pinocchioMapping, {"WRIST_2"}));

    // initial state of robot
    initState = mobileManipulatorInterfacePtr->getInitialState();
    // goal end-effector state
    goalState.setZero(7);
    goalState << 5.0, 5.0, 0.71, 1.0, 0.0, 0.0, 0.0;

    // current end-effector state
    const auto& model = pinocchioInterface->getModel();
    auto& data = pinocchioInterface->getData();
    const auto q = pinocchioMapping.getPinocchioJointPosition(initState);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    eeKinematicsPtr->setPinocchioInterface(*pinocchioInterface);
    const auto& curr_ee_positions = eeKinematicsPtr->getPosition(initState);
    const auto& curr_ee_orientation = data.oMf[model.getBodyId("WRIST_2")].rotation();

    // initialize reference
//    targetTrajectories.timeTrajectory.push_back(initTime);
    targetTrajectories.timeTrajectory.push_back(initTime + 2.0);
//    targetTrajectories.stateTrajectory.push_back(goalState);
    targetTrajectories.stateTrajectory.push_back(goalState);
//    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  }

  const scalar_t tolerance = 2e-2;
  const scalar_t f_mpc = 10.0;
  const scalar_t mpcIncrement = 1.0 / f_mpc;
  const scalar_t initTime = 1234.5;  // start from a random time
  const scalar_t finalTime = initTime + 5.0;

  vector_t initState;
  vector_t goalState;
  std::unique_ptr<MobileManipulatorInterface> mobileManipulatorInterfacePtr;
  TargetTrajectories targetTrajectories;
  std::unique_ptr<PinocchioInterface> pinocchioInterface;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  MobileManipulatorPinocchioMapping<scalar_t> pinocchioMapping;
};

TEST_F(MobileManipulatorIntegrationTest, synchronousTracking) {
  auto mpcPtr = mobileManipulatorInterfacePtr->getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.getReferenceManager().setTargetTrajectories(targetTrajectories);

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
  // resolve current end-effector state
  const auto& model = pinocchioInterface->getModel();
  auto& data = pinocchioInterface->getData();
  const auto q = pinocchioMapping.getPinocchioJointPosition(observation.state);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterface);
  const auto& curr_ee_positions = eeKinematicsPtr->getPosition(observation.state);
  std::cerr << "Current: " << curr_ee_positions[0].transpose() << '\n';
  std::cerr << "Goal: " << goalState.transpose() << '\n';
  // check that goal position is reached
  ASSERT_NEAR(curr_ee_positions[0](0), goalState(0), tolerance);
  ASSERT_NEAR(curr_ee_positions[0](1), goalState(1), tolerance);
}

TEST_F(MobileManipulatorIntegrationTest, coldStartMPC) {
  auto mpcPtr = mobileManipulatorInterfacePtr->getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.getReferenceManager().setTargetTrajectories(targetTrajectories);

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

  // resolve current end-effector state
  const auto& model = pinocchioInterface->getModel();
  auto& data = pinocchioInterface->getData();
  const auto q = pinocchioMapping.getPinocchioJointPosition(observation.state);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterface);
  const auto& curr_ee_positions = eeKinematicsPtr->getPosition(observation.state);
  // check that goal position is reached
  ASSERT_NEAR(curr_ee_positions[0](0), goalState(0), tolerance);
  ASSERT_NEAR(curr_ee_positions[0](1), goalState(1), tolerance);
}

TEST_F(MobileManipulatorIntegrationTest, asynchronousTracking) {
  auto mpcPtr = mobileManipulatorInterfacePtr->getMpc();
  MPC_MRT_Interface mpcInterface(*mpcPtr);
  mpcInterface.getReferenceManager().setTargetTrajectories(targetTrajectories);

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
