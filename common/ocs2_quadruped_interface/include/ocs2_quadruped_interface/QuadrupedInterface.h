/*
 * QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_oc/oc_solver/SolverSynchronizedModule.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_robotic_tools/common/RobotInterface.h>

#include <ocs2_switched_model_interface/core/ComModelBase.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>
#include <ocs2_switched_model_interface/core/ModelSettings.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>
#include <ocs2_switched_model_interface/cost/SwitchedModelCostBase.h>
#include <ocs2_switched_model_interface/dynamics/ComKinoSystemDynamicsAd.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

namespace switched_model {

class QuadrupedInterface : public ocs2::RobotInterface<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::RobotInterface<STATE_DIM, INPUT_DIM>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  using dimension_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = dimension_t::scalar_t;
  using scalar_array_t = dimension_t::scalar_array_t;
  using size_array_t = dimension_t::size_array_t;
  using state_vector_t = dimension_t::state_vector_t;
  using state_matrix_t = dimension_t::state_matrix_t;
  using input_matrix_t = dimension_t::input_matrix_t;

  using rollout_base_t = ocs2::RolloutBase<STATE_DIM, INPUT_DIM>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;

  using synchronized_module_t = ocs2::SolverSynchronizedModule<STATE_DIM, INPUT_DIM>;
  using synchronized_module_ptr_array_t = std::vector<std::shared_ptr<synchronized_module_t>>;

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

  std::shared_ptr<SwitchedModelModeScheduleManager> getModeScheduleManagerPtr() const { return modeScheduleManagerPtr_; }

  synchronized_module_ptr_array_t getSynchronizedModules() const { return solverModules_; };

  /** Gets kinematic model */
  const kinematic_model_t& getKinematicModel() const { return *kinematicModelPtr_; }

  /** Gets center of mass model */
  const com_model_t& getComModel() const { return *comModelPtr_; }

  /** Gets the loaded initial state */
  state_vector_t& getInitialState() { return initialState_; }
  const state_vector_t& getInitialState() const { return initialState_; }

  /** Gets the loaded initial partition times */
  const scalar_array_t& getInitialPartitionTimes() const { return partitioningTimes_; }

  /** Gets the loaded initial getInitialModeSequence */
  const ModeSequenceTemplate& getInitialModeSequence() const { return *defaultModeSequenceTemplate_; }

  /** Access to rollout settings */
  const ocs2::Rollout_Settings& rolloutSettings() const { return rolloutSettings_; }

  /** Access to model settings */
  const ModelSettings& modelSettings() const { return modelSettings_; };

  /** Gets the rollout class */
  const time_triggered_rollout_t& getRollout() const { return *timeTriggeredRolloutPtr_; }

  /** Gets the time horizon of MPC */
  scalar_t getTimeHorizon() const { return timeHorizon_; }

  const system_dynamics_t& getDynamics() const override { return *dynamicsPtr_; }

  const system_dynamics_derivative_t& getDynamicsDerivatives() const override { return *dynamicsDerivativesPtr_; }

  const cost_function_t& getCost() const override { return *costFunctionPtr_; }

  const constraint_t* getConstraintPtr() const override { return constraintsPtr_.get(); }

  const operating_point_t& getOperatingPoints() const override { return *operatingPointsPtr_; }

 private:
  /**
   * Load the settings from the path file.
   *
   * @param pathToConfigFile
   */
  void loadSettings(const std::string& pathToConfigFile);

 private:
  scalar_t timeHorizon_;
  ocs2::Rollout_Settings rolloutSettings_;
  ModelSettings modelSettings_;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;
  std::shared_ptr<SwitchedModelModeScheduleManager> modeScheduleManagerPtr_;
  synchronized_module_ptr_array_t solverModules_;

  std::unique_ptr<system_dynamics_t> dynamicsPtr_;
  std::unique_ptr<system_dynamics_derivative_t> dynamicsDerivativesPtr_;
  std::unique_ptr<constraint_t> constraintsPtr_;
  std::unique_ptr<cost_function_t> costFunctionPtr_;
  std::unique_ptr<operating_point_t> operatingPointsPtr_;
  std::unique_ptr<time_triggered_rollout_t> timeTriggeredRolloutPtr_;

  state_matrix_t Q_;
  input_matrix_t R_;
  state_matrix_t QFinal_;

  state_vector_t initialState_;
  scalar_array_t partitioningTimes_;
  std::unique_ptr<ModeSequenceTemplate> defaultModeSequenceTemplate_;
};

}  // end of namespace switched_model
