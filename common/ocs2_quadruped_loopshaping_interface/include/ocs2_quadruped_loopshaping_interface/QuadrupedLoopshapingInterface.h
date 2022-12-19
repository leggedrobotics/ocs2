//
// Created by rgrandia on 17.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>
#include <ocs2_robotic_tools/common/LoopshapingRobotInterface.h>

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/LoopshapingSynchronizedModule.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"

namespace switched_model_loopshaping {

class QuadrupedLoopshapingInterface : public ocs2::LoopshapingRobotInterface {
 public:
  using com_model_t = switched_model::ComModelBase<scalar_t>;
  using kinematic_model_t = switched_model::KinematicsModelBase<scalar_t>;

  using ad_com_model_t = switched_model::ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = switched_model::KinematicsModelBase<ad_scalar_t>;

  QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr,
                                std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition);

  ~QuadrupedLoopshapingInterface() override = default;

  const switched_model::QuadrupedInterface& getQuadrupedInterface() const { return this->get<switched_model::QuadrupedInterface>(); }

  std::shared_ptr<ocs2::LoopshapingSynchronizedModule> getLoopshapingSynchronizedModule() const { return loopshapingSynchronizedModule_; };

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return getQuadrupedInterface().getKinematicModel(); }

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return getQuadrupedInterface().getComModel(); }

  /** Gets the loaded initial state */
  const vector_t& getInitialState() const { return initialState_; }

  /** Access to model settings */
  const switched_model::ModelSettings& modelSettings() const { return getQuadrupedInterface().modelSettings(); };

  /** Access to model names */
  const std::vector<std::string>& getJointNames() const { return getQuadrupedInterface().getJointNames(); };
  const std::string& getBaseName() const { return getQuadrupedInterface().getBaseName(); };

  /** Gets the rollout class */
  const ocs2::RolloutBase& getRollout() const { return *timeTriggeredRolloutPtr_; }

 private:
  std::unique_ptr<ocs2::RolloutBase> timeTriggeredRolloutPtr_;
  std::shared_ptr<ocs2::LoopshapingSynchronizedModule> loopshapingSynchronizedModule_;

  vector_t initialState_;
};

}  // namespace switched_model_loopshaping
