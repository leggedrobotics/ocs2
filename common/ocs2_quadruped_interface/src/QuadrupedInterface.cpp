//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

#include <ocs2_core/misc/Display.h>
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
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
auto QuadrupedInterface::loadCostMatrices(const std::string& pathToConfigFile, const kinematic_model_t& kinematicModel,
                                          state_vector_t initialState) -> std::tuple<state_matrix_t, input_matrix_t, state_matrix_t> {
  state_matrix_t Q;
  input_matrix_t R;
  state_matrix_t QFinal;

  // cost function components
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q", Q);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "R", R);
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "Q_final", QFinal);

  // costs over Cartesian velocities
  Eigen::Matrix<scalar_t, 12, 12> J_allFeet;
  for (int leg = 0; leg < 4; ++leg) {
    Eigen::Matrix<double, 6, 12> J_thisfoot = kinematicModel.baseToFootJacobianInBaseFrame(leg, getJointPositions(initialState));
    J_allFeet.block<3, 12>(3 * leg, 0) = J_thisfoot.bottomRows<3>();
  }
  R.block<12, 12>(12, 12) = (J_allFeet.transpose() * R.block<12, 12>(12, 12) * J_allFeet).eval();
  return {Q, R, QFinal};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadrupedInterface::loadSettings(const std::string& pathToConfigFile) {
  rolloutSettings_.loadSettings(pathToConfigFile, "slq.rollout");
  modelSettings_ = loadModelSettings(pathToConfigFile);

  // partitioning times
  size_t numPartitions;
  ocs2::loadData::loadPartitioningTimes(pathToConfigFile, timeHorizon_, numPartitions, partitioningTimes_, true);

  // initial state of the switched system
  Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1> initRbdState;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState);
  SwitchedModelStateEstimator switchedModelStateEstimator(*comModelPtr_);
  initialState_ = switchedModelStateEstimator.estimateComkinoModelState(initRbdState);

  // load init mode schedule
  auto initialModeSequenceTemplate = loadModeSequenceTemplate(pathToConfigFile, "initialModeSequenceTemplate", false);
  size_array_t initModesSequence = initialModeSequenceTemplate.modeSequence;
  initModesSequence.push_back(string2ModeNumber("STANCE"));
  scalar_array_t initEventTimes(initialModeSequenceTemplate.switchingTimes.begin() + 1, initialModeSequenceTemplate.switchingTimes.end());
  ocs2::ModeSchedule initModeSchedule{initEventTimes, initModesSequence};

  // load the mode sequence template
  defaultModeSequenceTemplate_.reset(
      new ModeSequenceTemplate(loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false)));

  auto gaitSchedule = std::make_shared<GaitSchedule>(0.0, Gait{{}, {ModeNumber::STANCE}});
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(gaitSchedule));

  // Display
  std::cerr << "\nTime Partition: {" << ocs2::toDelimitedString(partitioningTimes_) << "}\n";
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
  std::cerr << "\nDefault Modes Sequence Template: \n" << *defaultModeSequenceTemplate_ << std::endl;
}

}  // namespace switched_model
