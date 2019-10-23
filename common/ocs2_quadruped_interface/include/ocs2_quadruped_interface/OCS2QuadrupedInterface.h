/*
 * OCS2QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

#ifndef OCS2QUADRUPEDINTERFACE_H_
#define OCS2QUADRUPEDINTERFACE_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <array>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ocs2_slq/SLQ.h>
#include <ocs2_slq/SLQ_MP.h>
#include <ocs2_slq/SLQ_Settings.h>

#include <ocs2_mpc/MPC_OCS2.h>
#include <ocs2_mpc/MPC_SLQ.h>

#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_robotic_tools/common/RobotInterfaceBase.h>

#include <ocs2_switched_model_interface/core/Model_Settings.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/core/SwitchedModelStateEstimator.h>
#include <ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h>
#include <ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class OCS2QuadrupedInterface : public ocs2::RobotInterfaceBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { state_dim_ = STATE_DIM, input_dim_ = INPUT_DIM, rbd_state_dim_ = 12 + 2 * JOINT_COORD_SIZE };

  using BASE = ocs2::RobotInterfaceBase<STATE_DIM, INPUT_DIM>;

  using Ptr = std::shared_ptr<OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>>;

  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;

  using state_estimator_t = SwitchedModelStateEstimator;

  using dimension_t = typename BASE::DIMENSIONS;
  using scalar_t = typename dimension_t::scalar_t;
  using scalar_array_t = typename dimension_t::scalar_array_t;
  using size_array_t = typename dimension_t::size_array_t;
  using state_vector_t = typename dimension_t::state_vector_t;
  using state_vector_array_t = typename dimension_t::state_vector_array_t;
  using state_vector_array2_t = typename dimension_t::state_vector_array2_t;
  using input_vector_t = typename dimension_t::input_vector_t;
  using input_vector_array_t = typename dimension_t::input_vector_array_t;
  using input_vector_array2_t = typename dimension_t::input_vector_array2_t;
  using eigen_scalar_t = typename dimension_t::eigen_scalar_t;
  using eigen_scalar_array_t = typename dimension_t::eigen_scalar_array_t;
  using state_matrix_t = typename dimension_t::state_matrix_t;
  using input_matrix_t = typename dimension_t::input_matrix_t;
  using input_state_matrix_t = typename dimension_t::input_state_matrix_t;
  using input_state_matrix_array_t = typename dimension_t::input_state_matrix_array_t;

  using cost_desired_trajectories_t = ocs2::CostDesiredTrajectories<scalar_t>;

  using rollout_base_t = ocs2::RolloutBase<STATE_DIM, INPUT_DIM>;
  using time_triggered_rollout_t = ocs2::TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;

  using rbd_state_vector_t = Eigen::Matrix<scalar_t, rbd_state_dim_, 1>;

  using cpg_t = SplineCPG<scalar_t>;
  using feet_z_planner_t = FeetZDirectionPlanner<scalar_t, cpg_t>;
  using feet_z_planner_ptr_t = typename feet_z_planner_t::Ptr;

  using logic_rules_t = SwitchedModelLogicRulesBase;
  using logic_rules_ptr_t = std::shared_ptr<logic_rules_t>;

  using mode_sequence_template_t = ocs2::ModeSequenceTemplate<scalar_t>;

  using slq_base_t = ocs2::SLQ_BASE<STATE_DIM, INPUT_DIM>;
  using slq_t = ocs2::SLQ<STATE_DIM, INPUT_DIM>;
  using slq_mp_t = ocs2::SLQ_MP<STATE_DIM, INPUT_DIM>;
  //  using ocs2_t = ocs2::OCS2Projected<STATE_DIM, INPUT_DIM>;
  using mpc_t = ocs2::MPC_SLQ<STATE_DIM, INPUT_DIM>;

  using slq_base_ptr_t = std::unique_ptr<slq_base_t>;
  using slq_ptr_t = std::unique_ptr<slq_t>;
  using slq_mp_ptr_t = std::unique_ptr<slq_mp_t>;
  //  using ocs2_ptr_t = std::unique_ptr<ocs2_t>;
  using mpc_ptr_t = std::unique_ptr<mpc_t>;

  using linear_controller_t = typename slq_base_t::linear_controller_t;
  using controller_ptr_array_t = typename slq_base_t::controller_ptr_array_t;
  using linear_controller_ptr_array_t = std::vector<linear_controller_t*>;
  using primal_solution_t = typename slq_base_t::primal_solution_t;

  using controlled_system_base_t = ocs2::ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using controlled_system_base_ptr_t = typename controlled_system_base_t::Ptr;

  /**
   * Default constructor
   */
  OCS2QuadrupedInterface() = delete;

  /**
   * Constructor
   *
   * @param pathToConfigFolder: Path to configuration folder
   */
  OCS2QuadrupedInterface(const kinematic_model_t& kinematicModel, const com_model_t& comModel, const std::string& pathToConfigFolder);

  /**
   * Destructor
   */
  virtual ~OCS2QuadrupedInterface() {}

  /**
   * setup all optimizes
   */
  virtual void setupOptimizer(const logic_rules_ptr_t& logicRulesPtr, const mode_sequence_template_t* modeSequenceTemplatePtr,
                              slq_base_ptr_t& slqPtr, mpc_ptr_t& mpcPtr) = 0;

  /** Gets the rollout class */
  virtual const rollout_base_t& getRollout() const = 0;

  /**
   * Run the SLQ algorithm.
   *
   * @param initTime: Initial time.
   * @param initState: Initial robot's RBD state.
   * @param finalTime: Final time.
   * @param initialControllersStock: Initial controller (optional).
   */
  void runSLQ(const scalar_t& initTime, const rbd_state_vector_t& initState, const scalar_t& finalTime,
              const linear_controller_ptr_array_t& initialControllersStock = linear_controller_ptr_array_t());

  std::shared_ptr<ocs2::HybridLogicRules> getLogicRulesPtr() override { return logicRulesPtr_; }

  /**
   * Updates the logicMachine.
   *
   * @param [in] partitioningTimes: partitioning times array.
   * @param [in] logicRulesUpdated: If the internal logicRules class has been already updated.
   */
  void updateLogicMachine(const scalar_array_t& partitioningTimes, bool logicRulesUpdated = true);

  /**
   * Computes switched model state from the RBD state
   *
   * @param [in] rbdState: RBD state
   * @param [out] comkinoState: Switched model state.
   */
  virtual void computeSwitchedModelState(const rbd_state_vector_t& rbdState, state_vector_t& comkinoState);

  /**
   * Computes the RBD state from the switched model state.
   *
   * @param [in] comkinoState: switched model state.
   * @param [in] comkinoInput: switched model input.
   * @param [out] rbdState: RBD state
   */
  void computeRbdModelState(const state_vector_t& comkinoState, const input_vector_t& comkinoInput, rbd_state_vector_t& rbdState);

  /**
   * Computes the CoM's local acceleration about the CoM frame.
   * Note that here the off-diagonal blocks of the inertia tensor is zero.
   *
   * @param [in] comkinoState: CoM-Kino model state.
   * @param [in] comkinoInput: CoM-Kino model input.
   * @param [out] comLocalAcceleration: CoM acceleration about the CoM frame.
   */
  void computeComLocalAcceleration(const state_vector_t& comkinoState, const input_vector_t& comkinoInput,
                                   base_coordinate_t& comLocalAcceleration);

  /**
   * Computes the CoM's pose, velocity, and acceleration in the origin frame from the switched model state and input.
   *
   * @param [in] comkinoState: CoM-Kino model state.
   * @param [in] comkinoInput: CoM-Kino model input.
   * @param [out] o_comPose: CoM pose in the origin frame.
   * @param [out] o_comVelocity: CoM velocity in the origin frame.
   * @param [out] o_comAcceleration: CoM acceleration in the origin frame.
   */
  void computeComStateInOrigin(const state_vector_t& comkinoState, const input_vector_t& comkinoInput, base_coordinate_t& o_comPose,
                               base_coordinate_t& o_comVelocity, base_coordinate_t& o_comAcceleration);

  /**
   * Estimates flat ground hight.
   *
   * @param [in] rbdState: RBD state.
   * @param [in] contactFlag: Contact flag.
   * @param [out] groundHight: Ground hight
   */
  void estimateFlatGround(const rbd_state_vector_t& rbdState, const contact_flag_t& contactFlag, scalar_t& groundHight) const;

  /**
   * Get a reference to the robot kinematic model.
   *
   * @return A reference to the robot kinematic model
   */
  const kinematic_model_t& getKinematicModel() const;

  /**
   * Get a reference to the robot CoM model.
   *
   * @return A reference to the robot CoM model
   */
  const com_model_t& getComModel() const;

  /**
   * Gets a reference to the internal SLQ class.
   *
   * @return Reference to the internal SLQ
   */
  slq_base_t& getSLQ();

  mpc_t& getMpc() override;

  /**
   * Gets the cost function and ISEs of the type-1 and type-2 constraints at the initial time.
   *
   * @param [out] costFunction: cost function value
   * @param [out] constraint1ISE: type-1 constraint ISE.
   * @param [out] constraint1ISE: type-2 constraint ISE.
   */
  void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constriantISE1, scalar_t& constriantISE2) const;

  /**
   * Gets Iterations Log of SLQ.
   *
   * @param [out] iterationCost: Each iteration's cost.
   * @param [out] iterationISE1: Each iteration's type-1 constraints ISE.
   * @param [out] iterationISE2: Each iteration's type-2 constraints ISE.
   */
  void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                        eigen_scalar_array_t& iterationISE2) const;

  /**
   * Gets optimized solution.
   *
   * @return The SLQ solution
   */
  primal_solution_t getPrimalSolution() const;

  /**
   * Get contact flags sequence
   *
   * @param [in] contactFlagsSequencePtr: a pointer to contactFlagsSequencePtr_
   */
  void getContactFlagsSequencePtr(const std::vector<contact_flag_t>*& contactFlagsSequencePtr) const;

  /**
   * Get Model settings.
   *
   * @return Model settings.
   */
  Model_Settings& modelSettings();

  /**
   * Gets the loaded initial RBD state.
   *
   * @param initRbdState: initial RBD state.
   */
  void getLoadedInitialState(rbd_state_vector_t& initRbdState) const;

  /**
   * Gets the time horizon of the MPC.
   *
   * @param [out] timeHorizon: The time horizon of the MPC.
   */
  void getLoadedTimeHorizon(scalar_t& timeHorizon) const;

  /**
   * Gets SLQ settings.
   *
   * @return SLQ settings
   */
  ocs2::SLQ_Settings& slqSettings();

 protected:
  /**
   * Load the settings from the path file.
   *
   * @param pathToConfigFile
   */
  void loadSettings(const std::string& pathToConfigFile) final;

  /**
   * Setups all optimizers which you require.
   *
   * @param [in] taskFile: Task's file full path.
   */
  void setupOptimizer(const std::string& taskFile) final { throw std::runtime_error("It has been overloaded."); }

  /*
   * Variables
   */
  ocs2::SLQ_Settings slqSettings_;
  ocs2::Rollout_Settings rolloutSettings_;

  std::unique_ptr<kinematic_model_t> kinematicModelPtr_;
  std::unique_ptr<com_model_t> comModelPtr_;

  state_estimator_t switchedModelStateEstimator_;

  logic_rules_ptr_t logicRulesPtr_;  // logic

  Model_Settings modelSettings_;

  state_matrix_t Q_;
  input_matrix_t R_;
  state_matrix_t QFinal_;
  state_vector_t xFinal_;

  scalar_t impulseWeight_;
  scalar_t impulseSigmeFactor_;

  rbd_state_vector_t initRbdState_;
  using BASE::initialState_;

  scalar_t initTime_;
  scalar_t finalTime_;
  scalar_t timeHorizon_;
  size_t numPartitions_;
  scalar_array_t partitioningTimes_;

  size_t initNumSubsystems_;
  scalar_array_t initEventTimes_;
  size_array_t initSwitchingModes_;
  std::vector<contact_flag_t> initStanceLegSequene_;

  size_t numSubsystems_;
  scalar_array_t switchingTimes_;
  size_array_t switchingModes_;
  std::vector<contact_flag_t> stanceLegSequene_;

  mode_sequence_template_t initialModeSequenceTemplate_;
  mode_sequence_template_t defaultModeSequenceTemplate_;

  scalar_t costFunction_;
  scalar_t constriantISE1_;
  scalar_t constriantISE2_;

  eigen_scalar_array_t iterationCost_;
  eigen_scalar_array_t iterationISE1_;
  eigen_scalar_array_t iterationISE2_;
  eigen_scalar_array_t ocs2Iterationcost_;

  // SLQ
  slq_base_ptr_t slqPtr_;
  // OCS2
  //	ocs2_ptr_t 		ocs2Ptr_;
  // MPC
  mpc_ptr_t mpcPtr_;

  primal_solution_t primalSolution_;
  std::vector<contact_flag_t> contactFlagsSequence_;
  cost_desired_trajectories_t costDesiredTrajectories_;

  ocs2::EigenLinearInterpolation<state_vector_t> linInterpolateState_;
  ocs2::EigenLinearInterpolation<input_vector_t> linInterpolateInput_;
  ocs2::EigenLinearInterpolation<input_vector_t> linInterpolateUff_;
  ocs2::EigenLinearInterpolation<input_state_matrix_t> linInterpolateK_;
};

}  // end of namespace switched_model

#include "implementation/OCS2QuadrupedInterface.h"

#endif /* OCS2QUADRUPEDINTERFACE_H_ */
