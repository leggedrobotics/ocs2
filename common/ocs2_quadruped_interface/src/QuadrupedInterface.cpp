//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadrupedInterface::QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                       const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)

    : kinematicModelPtr_(kinematicModel.clone()), comModelPtr_(comModel.clone()) {
  loadSettings(pathToConfigFolder + "/task.info");

  // Swing planner.
  switched_model::SwingTrajectoryPlannerSettings settings;
  settings.liftOffVelocity = 0.4;
  settings.touchDownVelocity = -0.2;
  settings.swingHeight = 0.1;
  settings.touchdownAfterHorizon = 0.2;
  settings.errorGain = 5.0;
  settings.swingTimeScale = 0.15;
  auto swingTrajectoryPlanner =
      std::make_shared<SwingTrajectoryPlanner>(settings, getComModel(), getKinematicModel(), logicRulesPtr_);
  solverModules_.push_back(swingTrajectoryPlanner);

  dynamicsPtr_.reset(new system_dynamics_t(adKinematicModel, adComModel, modelSettings_.recompileLibraries_));
  dynamicsDerivativesPtr_.reset(dynamicsPtr_->clone());
  constraintsPtr_.reset(new constraint_t(adKinematicModel, adComModel, modeScheduleManagerPtr_, swingTrajectoryPlanner, modelSettings_));
  costFunctionPtr_.reset(new cost_function_t(*comModelPtr_, modeScheduleManagerPtr_, Q_, R_, QFinal_));
  operatingPointsPtr_.reset(new operating_point_t(*comModelPtr_, modeScheduleManagerPtr_));
  timeTriggeredRolloutPtr_.reset(new time_triggered_rollout_t(*dynamicsPtr_, rolloutSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrupedInterface::loadSettings(const std::string& pathToConfigFile) {
  rolloutSettings_.loadSettings(pathToConfigFile, "slq.rollout");
  modelSettings_ = loadModelSettings(pathToConfigFile);

  std::cerr << std::endl;

  // partitioning times
  size_t numPartitions;
  ocs2::loadData::loadPartitioningTimes(pathToConfigFile, timeHorizon_, numPartitions, partitioningTimes_, true);

  // display
  std::cerr << "Time Partition: {";
  for (const auto& timePartition : partitioningTimes_) {
    std::cerr << timePartition << ", ";
  }
  std::cerr << "\b\b}" << std::endl;

  // initial state of the switched system
  Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1> initRbdState;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState);
  SwitchedModelStateEstimator switchedModelStateEstimator(*comModelPtr_);
  initialState_ = switchedModelStateEstimator.estimateComkinoModelState(initRbdState);

  // cost function components
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q", Q_);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "R", R_);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q_final", QFinal_);

  // costs over Cartesian velocities
  Eigen::Matrix<scalar_t, 12, 12> J_allFeet;
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Matrix<double, 6, 12> J_thisfoot = kinematicModelPtr_->baseToFootJacobianInBaseFrame(leg, getJointPositions(initRbdState));
    J_allFeet.block<3, 12>(3 * leg, 0) = J_thisfoot.bottomRows<3>();
  }
  R_.block<12, 12>(12, 12) = (J_allFeet.transpose() * R_.block<12, 12>(12, 12) * J_allFeet).eval();

  // load init mode schedule
  auto initialModeSequenceTemplate = loadModeSequenceTemplate(pathToConfigFile, "initialModeSequenceTemplate", false);
  size_array_t initModesSequence = initialModeSequenceTemplate.modeSequence;
  initModesSequence.push_back(string2ModeNumber("STANCE"));
  scalar_array_t initEventTimes(initialModeSequenceTemplate.switchingTimes.begin() + 1, initialModeSequenceTemplate.switchingTimes.end());
  ocs2::ModeSchedule initModeSchedule{initEventTimes, initModesSequence};
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;

  // load the mode sequence template
  defaultModeSequenceTemplate_.reset(
      new ModeSequenceTemplate(loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false)));
  std::cerr << "\nDefault Modes Sequence Template: \n" << *defaultModeSequenceTemplate_ << std::endl;

  auto logicRules = std::make_shared<GaitSchedule>(initModeSchedule, modelSettings_.phaseTransitionStanceTime_);
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(logicRules));
}

}  // namespace switched_model
