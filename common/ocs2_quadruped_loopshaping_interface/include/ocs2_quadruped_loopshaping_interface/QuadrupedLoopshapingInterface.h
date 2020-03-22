//
// Created by rgrandia on 17.02.20.
//

#pragma once

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

#include <ocs2_quadruped_interface/QuadrupedInterface.h>

#include "ocs2_quadruped_loopshaping_interface/LoopshapingDimensions.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingModeScheduleManager.h"
#include "ocs2_quadruped_loopshaping_interface/LoopshapingSynchronizedModule.h"

namespace switched_model_loopshaping {

class QuadrupedLoopshapingInterface : public ocs2::RobotInterface<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using system_dynamics_t =
      ocs2::LoopshapingDynamics<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using system_dynamics_derivative_t =
      ocs2::LoopshapingDynamicsDerivative<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using constraint_t =
      ocs2::LoopshapingConstraint<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using operating_point_t =
      ocs2::LoopshapingOperatingPoint<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using cost_function_t =
      ocs2::LoopshapingCost<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
  using filter_dynamics_t =
      ocs2::LoopshapingFilterDynamics<STATE_DIM, INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;

  using com_model_t = switched_model::ComModelBase<double>;
  using kinematic_model_t = switched_model::KinematicsModelBase<double>;
  using logic_rules_t = switched_model::GaitSchedule;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = switched_model::ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = switched_model::KinematicsModelBase<ad_scalar_t>;

  using dimension_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = dimension_t::scalar_t;
  using scalar_array_t = dimension_t::scalar_array_t;
  using size_array_t = dimension_t::size_array_t;
  using state_vector_t = dimension_t::state_vector_t;
  using state_matrix_t = dimension_t::state_matrix_t;
  using input_matrix_t = dimension_t::input_matrix_t;

  using rollout_base_t = ocs2::RolloutBase<STATE_DIM, INPUT_DIM>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;

  QuadrupedLoopshapingInterface(std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr, const std::string& pathToConfigFolder);

  ~QuadrupedLoopshapingInterface() override = default;

  std::shared_ptr<ocs2::LoopshapingDefinition> getLoopshapingDefinition() const { return loopshapingDefinition_; };

  std::shared_ptr<LoopshapingModeScheduleManager> getModeScheduleManagerPtr() const { return loopshapingModeScheduleManager_; }

  std::shared_ptr<LoopshapingSynchronizedModule> getLoopshapingSynchronizedModule() const { return loopshapingSynchronizedModule_; };

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return quadrupedPtr_->getKinematicModel(); };

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return quadrupedPtr_->getComModel(); };

  /** Gets the loaded initial state */
  const state_vector_t& getInitialState() const { return initialState_; }

  /** Gets the loaded initial partition times */
  const scalar_array_t& getInitialPartitionTimes() const { return quadrupedPtr_->getInitialPartitionTimes(); }

  /** Gets the loaded initial getInitialModeSequence */
  const switched_model::ModeSequenceTemplate& getInitialModeSequence() const { return quadrupedPtr_->getInitialModeSequence(); }

  /** Access to model settings */
  const switched_model::ModelSettings& modelSettings() const { return quadrupedPtr_->modelSettings(); };

  /** Gets the rollout class */
  const rollout_base_t& getRollout() const { return *timeTriggeredRolloutPtr_; }

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const operating_point_t& getOperatingPoints() const override { return *operatingPointsPtr_; }

 private:
  std::unique_ptr<switched_model::QuadrupedInterface> quadrupedPtr_;
  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;
  std::unique_ptr<filter_dynamics_t> filterDynamicsPtr_;
  std::shared_ptr<ocs2::LoopshapingDefinition> loopshapingDefinition_;
  std::shared_ptr<LoopshapingModeScheduleManager> loopshapingModeScheduleManager_;
  std::shared_ptr<LoopshapingSynchronizedModule> loopshapingSynchronizedModule_;

  state_vector_t initialState_;
};

}  // namespace switched_model_loopshaping
