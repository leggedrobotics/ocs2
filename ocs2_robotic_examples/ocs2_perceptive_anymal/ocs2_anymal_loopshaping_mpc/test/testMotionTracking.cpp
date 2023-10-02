//
// Created by rgrandia on 02.12.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_commands/LoadMotions.h>
#include <ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

class TestAnymalLoopshapingMpc : public ::testing::Test {
 public:
  TestAnymalLoopshapingMpc() {
    const auto model = anymal::AnymalModel::Camel;
    const std::string configName("c_series");
    const std::string path(__FILE__);
    const std::string dir = path.substr(0, path.find_last_of("/"));
    const std::string configFolder = dir + "/../config/" + configName;

    // Get interface
    anymalInterface = anymal::getAnymalLoopshapingInterface(anymal::getUrdfString(model), configFolder);

    problem = anymalInterface->getOptimalControlProblem();

    // Cost desired
    ocs2::scalar_t initTime = 0.0;
    ocs2::scalar_t finalTime = 1.0;
    const ocs2::vector_t systemState = anymalInterface->getInitialState().head(switched_model::STATE_DIM);
    const ocs2::vector_t state = anymalInterface->getInitialState();
    const ocs2::vector_t input = ocs2::vector_t::Zero(switched_model_loopshaping::INPUT_DIM);
    targetTrajectories = ocs2::TargetTrajectories{{initTime}, {systemState}, {input}};
    problem.targetTrajectoriesPtr = &targetTrajectories;

    const auto mpcSettings = ocs2::mpc::loadSettings(configFolder + "/task.info");
    const auto sqpSettings = ocs2::sqp::loadSettings(configFolder + "/multiple_shooting.info");
    mpcPtr = switched_model_loopshaping::getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);

    // Initialize
    anymalInterface->getReferenceManagerPtr()->preSolverRun(initTime, finalTime, state);
  }

  std::unique_ptr<ocs2::MPC_BASE> mpcPtr;
  std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> anymalInterface;
  ocs2::OptimalControlProblem problem;
  ocs2::TargetTrajectories targetTrajectories;
};

TEST_F(TestAnymalLoopshapingMpc, trot_in_place) {
  ocs2::scalar_t initTime = 0.0;
  ocs2::scalar_t finalTime = 1.0;
  ocs2::scalar_t f_mpc = 20;  // hz

  ocs2::MPC_MRT_Interface mpcInterface(*mpcPtr);

  ocs2::SystemObservation observation;
  observation.time = initTime;
  observation.state = anymalInterface->getInitialState();
  observation.input = ocs2::vector_t::Zero(switched_model_loopshaping::INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  switched_model::Gait gait;
  gait.duration = 1.0;
  gait.eventPhases = {0.45, 0.5, 0.95};
  using MN = switched_model::ModeNumber;
  gait.modeSequence = {MN::LF_RH, MN::STANCE, MN::RF_LH, MN::STANCE};
  anymalInterface->getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule()->setGaitAtTime(gait, initTime);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) {
    mpcInterface.advanceMpc();
  }

  // run MPC for N iterations
  auto time = initTime;
  while (time < finalTime) {
    std::cerr << time << "\n";

    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_mpc;

    size_t mode;
    ocs2::vector_t optimalState, optimalInput;

    mpcInterface.updatePolicy();
    std::cerr << mpcInterface.getPerformanceIndices() << "\n";
    mpcInterface.evaluatePolicy(time, ocs2::vector_t::Zero(switched_model_loopshaping::STATE_DIM), optimalState, optimalInput, mode);

    // use optimal state for the next observation:
    observation.time = time;
    observation.state = optimalState;
    observation.input = ocs2::vector_t::Zero(switched_model_loopshaping::INPUT_DIM);
    mpcInterface.setCurrentObservation(observation);
  }

  // Check if base tracking was achieved
  const switched_model::vector3_t finalBasePosition = observation.state.segment<3>(3);
  const switched_model::vector3_t finalRefBasePosition =
      ocs2::LinearInterpolation::interpolate(finalTime, targetTrajectories.timeTrajectory, targetTrajectories.stateTrajectory)
          .segment<3>(3);
  ASSERT_LT((finalBasePosition - finalRefBasePosition).norm(), 0.1);
}

TEST_F(TestAnymalLoopshapingMpc, motion_tracking) {
  const std::string path(__FILE__);
  const std::string dir = path.substr(0, path.find_last_of("/"));
  const std::string motionFilesPath = dir + "/../../ocs2_anymal_commands/config/motions/";
  const std::string motionName = "demo_motion";

  const auto csvData = switched_model::readCsv(motionFilesPath + motionName + ".txt");
  const auto motionData = switched_model::readMotion(csvData);

  ocs2::scalar_t initTime = motionData.first.timeTrajectory.front();
  ocs2::scalar_t finalTime = motionData.first.timeTrajectory.back();
  ocs2::scalar_t f_mpc = 100;  // hz

  ocs2::MPC_MRT_Interface mpcInterface(*mpcPtr);

  ocs2::SystemObservation observation;
  observation.time = initTime;
  observation.state = anymalInterface->getInitialState();
  observation.state.head(switched_model::STATE_DIM) =
      ocs2::LinearInterpolation::interpolate(initTime, motionData.first.timeTrajectory, motionData.first.stateTrajectory);
  observation.input = ocs2::vector_t::Zero(switched_model_loopshaping::INPUT_DIM);
  mpcInterface.setCurrentObservation(observation);

  mpcInterface.getReferenceManager().setTargetTrajectories(motionData.first);
  anymalInterface->getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr()->getGaitSchedule()->setGaitAtTime(motionData.second,
                                                                                                                      initTime);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) {
    mpcInterface.advanceMpc();
  }

  // run MPC for N iterations
  auto time = initTime;
  while (time < finalTime) {
    std::cerr << time << "\n";

    // run MPC
    mpcInterface.advanceMpc();
    time += 1.0 / f_mpc;

    size_t mode;
    ocs2::vector_t optimalState, optimalInput;

    mpcInterface.updatePolicy();
    std::cerr << mpcInterface.getPerformanceIndices() << "\n";
    mpcInterface.evaluatePolicy(time, ocs2::vector_t::Zero(switched_model_loopshaping::STATE_DIM), optimalState, optimalInput, mode);

    // use optimal state for the next observation:
    observation.time = time;
    observation.state = optimalState;
    observation.input = ocs2::vector_t::Zero(switched_model_loopshaping::INPUT_DIM);
    mpcInterface.setCurrentObservation(observation);
  }

  // Check if base tracking was achieved
  const switched_model::vector3_t finalBasePosition = observation.state.segment<3>(3);
  const switched_model::vector3_t finalRefBasePosition =
      ocs2::LinearInterpolation::interpolate(finalTime, motionData.first.timeTrajectory, motionData.first.stateTrajectory).segment<3>(3);
  ASSERT_LT((finalBasePosition - finalRefBasePosition).norm(), 0.25);
}