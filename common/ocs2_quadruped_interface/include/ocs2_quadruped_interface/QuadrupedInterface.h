/*
 * QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_ddp/SLQ_Settings.h>

#include <ocs2_mpc/MPC_OCS2.h>
#include <ocs2_mpc/MPC_SLQ.h>
#include <ocs2_mpc/MPC_Settings.h>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <ocs2_switched_model_interface/core/ModelSettings.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

namespace switched_model {

class QuadrupedInterface : public ocs2::RobotInterface<24, 24> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr auto joint_dim_ = 12;
  static constexpr auto state_dim_ = 24;
  static constexpr auto input_dim_ = 24;
  static constexpr auto rbd_state_dim_ = 12 + 2 * joint_dim_;

  using BASE = ocs2::RobotInterface<state_dim_, input_dim_>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;
  using logic_rules_t = SwitchedModelLogicRulesBase;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using dimension_t = ocs2::Dimensions<state_dim_, input_dim_>;
  using scalar_t = typename dimension_t::scalar_t;
  using scalar_array_t = typename dimension_t::scalar_array_t;
  using size_array_t = typename dimension_t::size_array_t;
  using state_vector_t = typename dimension_t::state_vector_t;
  using state_matrix_t = typename dimension_t::state_matrix_t;
  using input_matrix_t = typename dimension_t::input_matrix_t;

  using rollout_base_t = ocs2::RolloutBase<state_dim_, input_dim_>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<state_dim_, input_dim_>;

  using mode_sequence_template_t = ocs2::ModeSequenceTemplate<scalar_t>;

  using mpc_t = ocs2::MPC_SLQ<state_dim_, input_dim_>;
  using slq_t = ocs2::SLQ<state_dim_, input_dim_>;

  using system_dynamics_t = switched_model::ComKinoSystemDynamicsAd;
  using system_dynamics_derivative_t = switched_model::ComKinoSystemDynamicsAd;
  using constraint_t = switched_model::ComKinoConstraintBaseAd;
  using cost_function_t = switched_model::SwitchedModelCostBase;
  using operating_point_t = switched_model::ComKinoOperatingPointsBase;

  /**
   *
   * @param kinematicModel
   * @param comModel
   * @param pathToConfigFolder : Reads settings from the task.info in this folder
   */
  QuadrupedInterface(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                         const ad_com_model_t& adComModel, const std::string& pathToConfigFolder);
  
  /**
   * Destructor
   */
  ~QuadrupedInterface() override = default;

  std::shared_ptr<ocs2::HybridLogicRules> getLogicRulesPtr() const override { return logicRulesPtr_; }

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return *kinematicModelPtr_; };

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return *comModelPtr_; };

  /** Constructs an SLQ object */
  std::unique_ptr<slq_t> getSlq() const;

  /** Constructs an MPC object */
  std::unique_ptr<mpc_t> getMpc() const;

  /** Gets the loaded initial state */
  const state_vector_t& getInitialState() const { return initialState_; }

  /** Access to rollout settings */
  const ocs2::Rollout_Settings& rolloutSettings() const { return rolloutSettings_; };

  /** Access to model settings */
  const ModelSettings& modelSettings() const { return modelSettings_; };

  /** Access to slq settings */
  const ocs2::SLQ_Settings& slqSettings() const { return slqSettings_; }

  /** Access to mpc settings */
  const ocs2::MPC_Settings& mpcSettings() const { return mpcSettings_; }

  /** Gets the rollout class */
  const rollout_base_t& getRollout() const { return *timeTriggeredRolloutPtr_; }

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const operating_point_t& getOperatingPoint() const { return *operatingPointsPtr_; }

 private:
  /**
   * Load the settings from the path file.
   *
   * @param pathToConfigFile
   */
  void loadSettings(const std::string& pathToConfigFile);

 private:
  ocs2::SLQ_Settings slqSettings_;
  ocs2::MPC_Settings mpcSettings_;
  ocs2::Rollout_Settings rolloutSettings_;
  ModelSettings modelSettings_;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<ad_kinematic_model_t> adKinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;
  std::unique_ptr<ad_com_model_t> adComModelPtr_;
  std::shared_ptr<logic_rules_t> logicRulesPtr_;

  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<rollout_base_t> timeTriggeredRolloutPtr_;

  state_matrix_t Q_;
  input_matrix_t R_;
  state_matrix_t QFinal_;

  state_vector_t initialState_;

  scalar_array_t partitioningTimes_;
  mode_sequence_template_t defaultModeSequenceTemplate_;
};

}  // end of namespace switched_model

