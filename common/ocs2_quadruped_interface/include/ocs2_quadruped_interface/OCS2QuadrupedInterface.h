/*
 * OCS2QuadrupedInterface.h
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

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class OCS2QuadrupedInterface : public ocs2::RobotInterface<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr auto state_dim_ = STATE_DIM;
  static constexpr auto input_dim_ = INPUT_DIM;
  static constexpr auto rbd_state_dim_ = 12 + 2 * JOINT_COORD_SIZE;

  using BASE = ocs2::RobotInterface<STATE_DIM, INPUT_DIM>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;
  using logic_rules_t = SwitchedModelLogicRulesBase;

  using dimension_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename dimension_t::scalar_t;
  using scalar_array_t = typename dimension_t::scalar_array_t;
  using size_array_t = typename dimension_t::size_array_t;
  using state_vector_t = typename dimension_t::state_vector_t;
  using state_matrix_t = typename dimension_t::state_matrix_t;
  using input_matrix_t = typename dimension_t::input_matrix_t;

  using rollout_base_t = ocs2::RolloutBase<STATE_DIM, INPUT_DIM>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;

  using mode_sequence_template_t = ocs2::ModeSequenceTemplate<scalar_t>;

  using mpc_t = ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>;
  using slq_t = ocs2::SLQ<STATE_DIM, INPUT_DIM>;

  /**
   *
   * @param kinematicModel
   * @param comModel
   * @param pathToConfigFolder : Reads settings from the task.info in this folder
   */
  OCS2QuadrupedInterface(const kinematic_model_t& kinematicModel, const com_model_t& comModel, const std::string& pathToConfigFolder);

  /**
   * Destructor
   */
  ~OCS2QuadrupedInterface() override = default;

  std::shared_ptr<ocs2::HybridLogicRules> getLogicRulesPtr() const override { return logicRulesPtr_; }

  /** Gets the rollout class */
  virtual const rollout_base_t& getRollout() const = 0;

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return *kinematicModelPtr_; };

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return *comModelPtr_; };

  /** Constructs an SLQ object */
  virtual std::unique_ptr<slq_t> getSlq() const = 0;

  /** Constructs an MPC object */
  virtual std::unique_ptr<mpc_t> getMpc() const = 0;

  /** Gets the loaded initial state */
  const state_vector_t& getInitialState() const { return initialState_; }

  /** Access to model settings */
  const ModelSettings& modelSettings() const { return modelSettings_; };

  /** Access to slq settings */
  const ocs2::SLQ_Settings& slqSettings() const { return slqSettings_; }

  /** Access to mpc settings */
  const ocs2::MPC_Settings& mpcSettings() const { return mpcSettings_; }

 private:
  /**
   * Load the settings from the path file.
   *
   * @param pathToConfigFile
   */
  void loadSettings(const std::string& pathToConfigFile);

 protected:
  ocs2::SLQ_Settings slqSettings_;
  ocs2::MPC_Settings mpcSettings_;
  ocs2::Rollout_Settings rolloutSettings_;
  ModelSettings modelSettings_;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<logic_rules_t> logicRulesPtr_;

  state_matrix_t Q_;
  input_matrix_t R_;
  state_matrix_t QFinal_;

  state_vector_t initialState_;

  scalar_array_t partitioningTimes_;
  mode_sequence_template_t defaultModeSequenceTemplate_;
};

}  // end of namespace switched_model

#include "implementation/OCS2QuadrupedInterface.h"
