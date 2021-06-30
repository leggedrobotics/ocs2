//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_interface/terrain/PlanarTerrainModel.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrupedInterface::QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                       const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)

    : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()) {
  loadSettings(pathToConfigFolder + "/task.info");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrupedInterface::loadSettings(const std::string& pathToConfigFile) {
  rolloutSettings_ = ocs2::rollout::loadSettings(pathToConfigFile, "rollout");
  modelSettings_ = loadModelSettings(pathToConfigFile);
  trackingWeights_ = loadWeightsFromFile(pathToConfigFile, "tracking_cost_weights");

  // initial state of the switched system
  Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1> initRbdState;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState);
  SwitchedModelStateEstimator switchedModelStateEstimator(*comModelPtr_);
  initialState_ = switchedModelStateEstimator.estimateComkinoModelState(initRbdState);

  // Gait Schedule
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false);
  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  std::unique_ptr<GaitSchedule> gaitSchedule(new GaitSchedule(0.0, defaultGait));

  // Swing trajectory planner
  const auto swingTrajectorySettings = loadSwingTrajectorySettings(pathToConfigFile);
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(swingTrajectorySettings, getComModel(), getKinematicModel()));

  // Terrain
  auto loadedTerrain = loadTerrainPlane(pathToConfigFile, true);
  std::unique_ptr<TerrainModel> terrainModel(new PlanarTerrainModel(std::move(loadedTerrain)));

  // Mode schedule manager
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(gaitSchedule), std::move(swingTrajectoryPlanner),
                                                                               std::move(terrainModel));

  // Display
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;
}

}  // namespace switched_model
