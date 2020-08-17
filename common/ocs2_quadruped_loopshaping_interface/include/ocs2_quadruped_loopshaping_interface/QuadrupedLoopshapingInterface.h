//
// Created by rgrandia on 17.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_switched_model_interface/Dimensions.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingModeScheduleManager.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingSynchronizedModule.h"

namespace switched_model_loopshaping {

class QuadrupedLoopshapingInterface : public ocs2::RobotInterface {
 public:
  using com_model_t = switched_model::ComModelBase<double>;
  using kinematic_model_t = switched_model::KinematicsModelBase<double>;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = switched_model::ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = switched_model::KinematicsModelBase<ad_scalar_t>;

  QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr, const std::string& pathToConfigFolder);

  ~QuadrupedLoopshapingInterface() override = default;

  std::shared_ptr<ocs2::LoopshapingDefinition> getLoopshapingDefinition() const { return loopshapingDefinition_; };

  std::shared_ptr<LoopshapingModeScheduleManager> getModeScheduleManagerPtr() const { return loopshapingModeScheduleManager_; }

  std::shared_ptr<LoopshapingSynchronizedModule> getLoopshapingSynchronizedModule() const { return loopshapingSynchronizedModule_; };

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return quadrupedPtr_->getKinematicModel(); }

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return quadrupedPtr_->getComModel(); }

  /** Gets the loaded initial state */
  const vector_t& getInitialState() const { return initialState_; }

  /** Gets the loaded initial partition times */
  const scalar_array_t& getInitialPartitionTimes() const { return quadrupedPtr_->getInitialPartitionTimes(); }

  /** Access to model settings */
  const switched_model::ModelSettings& modelSettings() const { return quadrupedPtr_->modelSettings(); };

  /** Gets the rollout class */
  const ocs2::RolloutBase& getRollout() const { return *timeTriggeredRolloutPtr_; }

  const ocs2::LoopshapingDynamics& getDynamics() const override { return *dynamicsPtr_; }

  const ocs2::LoopshapingCost& getCost() const override { return *costFunctionPtr_; }

  const ocs2::LoopshapingConstraint* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const ocs2::LoopshapingOperatingPoint& getOperatingPoints() const override { return *operatingPointsPtr_; }

 private:
  std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr_;
  std::unique_ptr<ocs2::LoopshapingDynamics> dynamicsPtr_;
  std::unique_ptr<ocs2::LoopshapingConstraint> constraintsPtr_;
  std::unique_ptr<ocs2::LoopshapingCost> costFunctionPtr_;
  std::unique_ptr<ocs2::LoopshapingOperatingPoint> operatingPointsPtr_;
  std::unique_ptr<ocs2::RolloutBase> timeTriggeredRolloutPtr_;
  std::unique_ptr<ocs2::LoopshapingFilterDynamics> filterDynamicsPtr_;
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::shared_ptr<LoopshapingModeScheduleManager> loopshapingModeScheduleManager_;
  std::shared_ptr<LoopshapingSynchronizedModule> loopshapingSynchronizedModule_;

  vector_t initialState_;
};

}  // namespace switched_model_loopshaping
