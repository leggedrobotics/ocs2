//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityInFootFrameConstraint.h>
#include <ocs2_switched_model_interface/constraint/FootNormalConstraint.h>
#include <ocs2_switched_model_interface/constraint/FrictionConeConstraint.h>
#include <ocs2_switched_model_interface/constraint/ZeroForceConstraint.h>
#include <ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/cost/CollisionAvoidanceCost.h>
#include <ocs2_switched_model_interface/cost/FootPlacementCost.h>
#include <ocs2_switched_model_interface/cost/FrictionConeCost.h>
#include <ocs2_switched_model_interface/cost/JointLimitsSoftConstraint.h>
#include <ocs2_switched_model_interface/cost/MotionTrackingCost.h>
#include <ocs2_switched_model_interface/cost/TorqueLimitsSoftConstraint.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h>
#include <ocs2_switched_model_interface/initialization/ComKinoInitializer.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_interface/terrain/PlanarTerrainModel.h>

namespace switched_model {

QuadrupedInterface::QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel,
                                       const com_model_t& comModel, const ad_com_model_t& adComModel, const std::string& pathToConfigFolder)

    : kinematicModelPtr_(kinematicModel.clone()),
      adKinematicModelPtr_(adKinematicModel.clone()),
      comModelPtr_(comModel.clone()),
      adComModelPtr_(adComModel.clone()),
      problemPtr_(new ocs2::OptimalControlProblem) {
  loadSettings(pathToConfigFolder + "/task.info");
}

void QuadrupedInterface::loadSettings(const std::string& pathToConfigFile) {
  rolloutSettings_ = ocs2::rollout::loadSettings(pathToConfigFile, "rollout");
  modelSettings_ = loadModelSettings(pathToConfigFile);
  trackingWeights_ = loadWeightsFromFile(pathToConfigFile, "tracking_cost_weights");

  // initial state of the switched system
  Eigen::Matrix<scalar_t, RBD_STATE_DIM, 1> initRbdState;
  ocs2::loadData::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState);
  initialState_ = estimateComkinoModelState(initRbdState);

  // Gait Schedule
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(pathToConfigFile, "defaultModeSequenceTemplate", false);
  const auto defaultGait = toGait(defaultModeSequenceTemplate);
  std::unique_ptr<GaitSchedule> gaitSchedule(new GaitSchedule(0.0, defaultGait));

  // Swing trajectory planner
  const auto swingTrajectorySettings = loadSwingTrajectorySettings(pathToConfigFile);
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(swingTrajectorySettings, getKinematicModel()));

  // Terrain
  auto loadedTerrain = loadTerrainPlane(pathToConfigFile, true);
  std::unique_ptr<TerrainModel> terrainModel(new PlanarTerrainModel(std::move(loadedTerrain)));

  // Mode schedule manager
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(gaitSchedule), std::move(swingTrajectoryPlanner),
                                                                               std::move(terrainModel));

  // Display
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;
}

std::unique_ptr<ocs2::PreComputation> QuadrupedInterface::createPrecomputation() const {
  return std::unique_ptr<ocs2::PreComputation>(
      new SwitchedModelPreComputation(getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(), getKinematicModel(),
                                      getKinematicModelAd(), getComModel(), getComModelAd(), modelSettings()));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createMotionTrackingCost() const {
  return std::unique_ptr<ocs2::StateInputCost>(new MotionTrackingCost(
      costSettings(), *getSwitchedModelModeScheduleManagerPtr(), getSwitchedModelModeScheduleManagerPtr()->getSwingTrajectoryPlanner(),
      getKinematicModel(), getKinematicModelAd(), getComModel(), getComModelAd(), modelSettings().recompileLibraries_));
}

std::unique_ptr<ocs2::StateCost> QuadrupedInterface::createFootPlacementCost() const {
  return std::unique_ptr<ocs2::StateCost>(
      new FootPlacementCost(ocs2::RelaxedBarrierPenalty::Config(modelSettings().muFootPlacement_, modelSettings().deltaFootPlacement_)));
}

std::unique_ptr<ocs2::StateCost> QuadrupedInterface::createCollisionAvoidanceCost() const {
  return std::unique_ptr<ocs2::StateCost>(
      new CollisionAvoidanceCost(ocs2::RelaxedBarrierPenalty::Config(modelSettings().muSdf_, modelSettings().deltaSdf_)));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createJointLimitsSoftConstraint() const {
  return std::unique_ptr<ocs2::StateInputCost>(new JointLimitsSoftConstraint(
      {modelSettings().lowerJointLimits_, modelSettings().upperJointLimits_}, modelSettings().jointVelocityLimits,
      {modelSettings().muJointsPosition_, modelSettings().deltaJointsPosition_},
      {modelSettings().muJointsVelocity_, modelSettings().muJointsVelocity_}));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createTorqueLimitsSoftConstraint(const joint_coordinate_t& nominalTorques) const {
  return std::unique_ptr<ocs2::StateInputCost>(new TorqueLimitsSoftConstraint(
      modelSettings().jointTorqueLimits, {modelSettings().muJointsTorque_, modelSettings().deltaJointsTorque_}, nominalTorques));
}

std::unique_ptr<ocs2::SystemDynamicsBase> QuadrupedInterface::createDynamics() const {
  return std::unique_ptr<ocs2::SystemDynamicsBase>(
      new ComKinoSystemDynamicsAd(getKinematicModelAd(), getComModelAd(), *getSwitchedModelModeScheduleManagerPtr(), modelSettings()));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createZeroForceConstraint(size_t leg) const {
  return std::unique_ptr<ocs2::StateInputConstraint>(new ZeroForceConstraint(leg, *getSwitchedModelModeScheduleManagerPtr()));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createFootNormalConstraint(size_t leg) const {
  return std::unique_ptr<ocs2::StateInputConstraint>(new FootNormalConstraint(leg));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createEndEffectorVelocityConstraint(size_t leg) const {
  return std::unique_ptr<ocs2::StateInputConstraint>(new EndEffectorVelocityConstraint(leg, *getSwitchedModelModeScheduleManagerPtr()));
}

std::unique_ptr<ocs2::StateInputConstraint> QuadrupedInterface::createEndEffectorVelocityInFootFrameConstraint(size_t leg) const {
  return std::unique_ptr<ocs2::StateInputConstraint>(new EndEffectorVelocityInFootFrameConstraint(
      leg, *getSwitchedModelModeScheduleManagerPtr(), getKinematicModelAd(), modelSettings().recompileLibraries_));
}

std::unique_ptr<ocs2::StateInputCost> QuadrupedInterface::createFrictionConeCost(size_t leg) const {
  FrictionConeConstraint::Config frictionConfig(modelSettings().frictionCoefficient_);
  std::unique_ptr<ocs2::PenaltyBase> penalty(
      new ocs2::RelaxedBarrierPenalty({modelSettings().muFrictionCone_, modelSettings().deltaFrictionCone_}));
  return std::unique_ptr<ocs2::StateInputCost>(
      new FrictionConeCost(std::move(frictionConfig), leg, *getSwitchedModelModeScheduleManagerPtr(), std::move(penalty)));
}

}  // namespace switched_model
