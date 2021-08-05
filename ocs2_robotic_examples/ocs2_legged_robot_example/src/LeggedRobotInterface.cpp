/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_legged_robot_example/LeggedRobotInterface.h"

#include <ros/package.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFileFolderName, const std::string& targetCommandFile,
                                           const ::urdf::ModelInterfaceSharedPtr& urdfTree) {
  // Load the task file
  const std::string taskFolder = ros::package::getPath("ocs2_legged_robot_example") + "/config/" + taskFileFolderName;
  const std::string taskFile = taskFolder + "/task.info";
  std::cerr << "Loading task file: " << taskFile << std::endl;

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile);
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile);
  rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout");

  // OptimalConrolProblem
  setupOptimalConrolProblem(taskFile, targetCommandFile, urdfTree);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& taskFile) {
  const auto initModeSchedule = loadModeSchedule(taskFile, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(taskFile, "defaultModeSequenceTemplate", false);

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

  // display
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimalConrolProblem(const std::string& taskFile, const std::string& targetCommandFile,
                                                     const ::urdf::ModelInterfaceSharedPtr& urdfTree) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(
      new PinocchioInterface(ocs2::centroidal_model::createPinocchioInterface(urdfTree, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(12, targetCommandFile), modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config"), 4));

  // Mode schedule manager
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(taskFile), std::move(swingTrajectoryPlanner));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));

  // Cost function
  costPtr_.reset(new LeggedRobotCost(*pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_, taskFile, modelSettings_));

  // Constraints
  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  if (useAnalyticalGradientsConstraints) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimizer] The analytical constraint class is not yet implemented.");
  } else {
    constraintsPtr_.reset(new LeggedRobotConstraintAD(*pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_,
                                                      *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_));
  }

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimizer] The analytical dynamics class is not yet implemented.");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr_.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<ocs2::MPC_DDP> LeggedRobotInterface::getMpcPtr() const {
  std::unique_ptr<ocs2::MPC_DDP> mpcPtr(new ocs2::MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintsPtr_.get(), costPtr_.get(),
                                                          initializerPtr_.get(), ddpSettings_, mpcSettings_));
  return mpcPtr;
}

}  // namespace legged_robot
}  // namespace ocs2
