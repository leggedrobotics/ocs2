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

TEST(TestAnymalLoopshapingMotionTracking, testSensitivity) {
  const auto model = anymal::AnymalModel::Camel;
  const std::string configName("c_series");
  const std::string motionName = "walking";
  const std::string path(__FILE__);
  const std::string dir = path.substr(0, path.find_last_of("/"));
  const std::string configFolder = dir + "/../config/" + configName;
  const std::string motionFilesPath = dir + "/../../ocs2_anymal_commands/config/motions/";

  const auto csvData = switched_model::readCsv(motionFilesPath + motionName + ".txt");
  const auto motionData = switched_model::readMotion(csvData, 0.05);
  const auto mpcSettings = ocs2::mpc::loadSettings(configFolder + "/task.info");
  const auto sqpSettings = ocs2::sqp::loadSettings(configFolder + "/multiple_shooting.info");
  auto quadrupedSettings = switched_model::loadQuadrupedSettings(configFolder + "/task.info");
  const auto frameDecl = anymal::frameDeclarationFromFile(configFolder + "/frame_declaration.info");
  auto loopshapingDefinition = ocs2::loopshaping_property_tree::load(configFolder + "/loopshaping.info");

  ocs2::scalar_t initTime = motionData.first.timeTrajectory.front();
  ocs2::scalar_t finalTime = motionData.first.timeTrajectory.back();
  ocs2::scalar_t f_mpc = 100;  // hz

  auto pertubation = [](switched_model::vector3_t& weights, switched_model::scalar_t scale) {
    weights += scale * weights.cwiseProduct(switched_model::vector3_t::Random());
  };

  switched_model::scalar_t initPertubation = 0.0;
  pertubation(quadrupedSettings.trackingWeights_.eulerXYZ, initPertubation);
  pertubation(quadrupedSettings.trackingWeights_.comPosition, initPertubation);
  pertubation(quadrupedSettings.trackingWeights_.comAngularVelocity, initPertubation);
  pertubation(quadrupedSettings.trackingWeights_.comLinearVelocity, initPertubation);
  for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
    pertubation(quadrupedSettings.trackingWeights_.jointPosition[leg], initPertubation);
    pertubation(quadrupedSettings.trackingWeights_.footPosition[leg], initPertubation);
    pertubation(quadrupedSettings.trackingWeights_.jointVelocity[leg], initPertubation);
    pertubation(quadrupedSettings.trackingWeights_.footVelocity[leg], initPertubation);
    pertubation(quadrupedSettings.trackingWeights_.contactForce[leg], initPertubation);
  }

  //  switched_model::vector_t fixedPertubation(72);
  //  fixedPertubation << -0.2637, 0.5929, 0.2175, 1.4130, 0.6363, 0.0801, 0.2722, 0.1047, 0.3784, -2.8149, 0.0432, 0.8614, 0.3006, -0.5959,
  //      0.0476, -0.0116, 0.0550, 0.3269, 0.1988, -0.7673, -0.2221, 0.1776, -0.0030, -0.2777, -0.7968, -0.0891, -0.0418, -0.3855, 0.3938,
  //      0.0153, 0.2518, 0.4857, 0.0738, -0.1309, 0.6136, -0.2562, 0.0193, 0.0311, 0.0934, -0.0002, 0.1328, -0.0119, -0.1210, -0.0163,
  //      -0.0230, 0.0221, -0.0103, -0.0037, -0.4827, -0.4070, 0.2466, 0.0855, 0.4314, 0.0678, 1.1602, 0.0036, -0.1345, -0.3520, 0.4472,
  //      -0.3660, -0.6880, 0.5961, 0.5977, -0.2640, 0.3858, -0.4024, -0.7835, -0.4275, 0.8153, -0.8321, 0.0672, 0.1678;
  //  fixedPertubation.array() += 1.0;
  //  fixedPertubation.setOnes(72);
  //  int idx = 0;
  //  quadrupedSettings.trackingWeights_.eulerXYZ.x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.eulerXYZ.y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.eulerXYZ.z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comPosition.x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comPosition.y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comPosition.z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comAngularVelocity.x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comAngularVelocity.y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comAngularVelocity.z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comLinearVelocity.x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comLinearVelocity.y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.comLinearVelocity.z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[0].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[0].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[0].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[1].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[1].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[1].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[2].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[2].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[2].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[3].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[3].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointPosition[3].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[0].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[0].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[0].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[1].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[1].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[1].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[2].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[2].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[2].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[3].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[3].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footPosition[3].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[0].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[0].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[0].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[1].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[1].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[1].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[2].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[2].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[2].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[3].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[3].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.jointVelocity[3].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[0].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[0].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[0].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[1].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[1].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[1].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[2].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[2].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[2].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[3].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[3].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.footVelocity[3].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[0].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[0].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[0].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[1].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[1].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[1].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[2].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[2].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[2].z() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[3].x() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[3].y() *= fixedPertubation[idx++];
  //  quadrupedSettings.trackingWeights_.contactForce[3].z() *= fixedPertubation[idx++];

  int Nruns = 1;
  int dataSize = switched_model::STATE_DIM + switched_model::INPUT_DIM + 3 * 4 + switched_model::NUM_CONTACT_POINTS * 3 * 5;
  ocs2::matrix_t results(dataSize, Nruns);
  for (int run = 0; run < Nruns; ++run) {
    auto quadrupedSettingsCopy = quadrupedSettings;

    switched_model::scalar_t runningPertubation = 0.0;
    pertubation(quadrupedSettingsCopy.trackingWeights_.eulerXYZ, runningPertubation);
    pertubation(quadrupedSettingsCopy.trackingWeights_.comPosition, runningPertubation);
    pertubation(quadrupedSettingsCopy.trackingWeights_.comAngularVelocity, runningPertubation);
    pertubation(quadrupedSettingsCopy.trackingWeights_.comLinearVelocity, runningPertubation);
    for (size_t leg = 0; leg < switched_model::NUM_CONTACT_POINTS; ++leg) {
      pertubation(quadrupedSettingsCopy.trackingWeights_.jointPosition[leg], runningPertubation);
      pertubation(quadrupedSettingsCopy.trackingWeights_.footPosition[leg], runningPertubation);
      pertubation(quadrupedSettingsCopy.trackingWeights_.jointVelocity[leg], runningPertubation);
      pertubation(quadrupedSettingsCopy.trackingWeights_.footVelocity[leg], runningPertubation);
      pertubation(quadrupedSettingsCopy.trackingWeights_.contactForce[leg], runningPertubation);
    }

    // Get interface
    auto anymalInterface = anymal::getAnymalLoopshapingInterface(anymal::getUrdfString(model), quadrupedSettingsCopy, frameDecl, loopshapingDefinition);

    auto mpcPtr = switched_model_loopshaping::getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
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
    std::cout << run << "\n";
    auto time = initTime;
    switched_model::comkino_state_t stateSSE = switched_model::comkino_state_t::Zero();
    switched_model::comkino_state_t inputSSE = switched_model::comkino_input_t::Zero();
    while (time < finalTime) {
      // run MPC
      mpcInterface.advanceMpc();
      time += 1.0 / f_mpc;

      size_t mode;
      ocs2::vector_t optimalState, optimalInput;

      mpcInterface.updatePolicy();
      mpcInterface.evaluatePolicy(time, ocs2::vector_t::Zero(switched_model_loopshaping::STATE_DIM), optimalState, optimalInput, mode);

      // use optimal state for the next observation:
      observation.time = time;
      observation.state = optimalState;
      observation.input = optimalInput;
      mpcInterface.setCurrentObservation(observation);

      // Base tracking
      const auto refState = ocs2::LinearInterpolation::interpolate(time, motionData.first.timeTrajectory, motionData.first.stateTrajectory);
      const auto refInput = ocs2::LinearInterpolation::interpolate(time, motionData.first.timeTrajectory, motionData.first.inputTrajectory);
      stateSSE += 1.0 / f_mpc * (observation.state.head<switched_model::STATE_DIM>() - refState).cwiseAbs2();
      inputSSE += 1.0 / f_mpc * (observation.input - refInput).cwiseAbs2();
    }

    stateSSE /= (finalTime - initTime);
    stateSSE = stateSSE.cwiseSqrt();
    inputSSE /= (finalTime - initTime);
    inputSSE = inputSSE.cwiseSqrt();

    // Tracking performance vs. weights
    ocs2::vector_t dataLine(dataSize);
    dataLine << stateSSE, inputSSE, quadrupedSettingsCopy.trackingWeights_.eulerXYZ, quadrupedSettingsCopy.trackingWeights_.comPosition,
        quadrupedSettingsCopy.trackingWeights_.comAngularVelocity, quadrupedSettingsCopy.trackingWeights_.comLinearVelocity,
        quadrupedSettingsCopy.trackingWeights_.jointPosition[0], quadrupedSettingsCopy.trackingWeights_.jointPosition[1],
        quadrupedSettingsCopy.trackingWeights_.jointPosition[2], quadrupedSettingsCopy.trackingWeights_.jointPosition[3],
        quadrupedSettingsCopy.trackingWeights_.footPosition[0], quadrupedSettingsCopy.trackingWeights_.footPosition[1],
        quadrupedSettingsCopy.trackingWeights_.footPosition[2], quadrupedSettingsCopy.trackingWeights_.footPosition[3],
        quadrupedSettingsCopy.trackingWeights_.jointVelocity[0], quadrupedSettingsCopy.trackingWeights_.jointVelocity[1],
        quadrupedSettingsCopy.trackingWeights_.jointVelocity[2], quadrupedSettingsCopy.trackingWeights_.jointVelocity[3],
        quadrupedSettingsCopy.trackingWeights_.footVelocity[0], quadrupedSettingsCopy.trackingWeights_.footVelocity[1],
        quadrupedSettingsCopy.trackingWeights_.footVelocity[2], quadrupedSettingsCopy.trackingWeights_.footVelocity[3],
        quadrupedSettingsCopy.trackingWeights_.contactForce[0], quadrupedSettingsCopy.trackingWeights_.contactForce[1],
        quadrupedSettingsCopy.trackingWeights_.contactForce[2], quadrupedSettingsCopy.trackingWeights_.contactForce[3];
    results.col(run) = dataLine;
  }

  Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
  std::cerr << results.transpose().format(CommaFmt);
}