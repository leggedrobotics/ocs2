/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#ifndef DDP_BASE_OCS2_H_
#define DDP_BASE_OCS2_H_

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/TrajectorySpreadingController.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/rollout/OperatingTrajectoriesRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_ddp_base/DDP_Settings.h>

#define BENCHMARK

namespace ocs2 {

/**
 * This class is an interface class for the DDP based methods.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class DDP_BASE : public Solver_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Solver_BASE<STATE_DIM, INPUT_DIM> BASE;

  using DIMENSIONS = typename BASE::DIMENSIONS;
  using controller_t = typename BASE::controller_t;
  using size_array_t = typename BASE::size_array_t;
  using size_array2_t = typename BASE::size_array2_t;
  using scalar_t = typename BASE::scalar_t;
  using scalar_array_t = typename BASE::scalar_array_t;
  using scalar_array2_t = typename BASE::scalar_array2_t;
  using scalar_array3_t = typename BASE::scalar_array3_t;
  using eigen_scalar_t = typename BASE::eigen_scalar_t;
  using eigen_scalar_array_t = typename BASE::eigen_scalar_array_t;
  using eigen_scalar_array2_t = typename BASE::eigen_scalar_array2_t;
  using state_vector_t = typename BASE::state_vector_t;
  using state_vector_array_t = typename BASE::state_vector_array_t;
  using state_vector_array2_t = typename BASE::state_vector_array2_t;
  using state_vector_array3_t = typename BASE::state_vector_array3_t;
  using input_vector_t = typename BASE::input_vector_t;
  using input_vector_array_t = typename BASE::input_vector_array_t;
  using input_vector_array2_t = typename BASE::input_vector_array2_t;
  using input_vector_array3_t = typename BASE::input_vector_array3_t;
  using input_state_matrix_t = typename BASE::input_state_matrix_t;
  using input_state_matrix_array_t = typename BASE::input_state_matrix_array_t;
  using input_state_matrix_array2_t = typename BASE::input_state_matrix_array2_t;
  using input_state_matrix_array3_t = typename BASE::input_state_matrix_array3_t;
  using state_matrix_t = typename BASE::state_matrix_t;
  using state_matrix_array_t = typename BASE::state_matrix_array_t;
  using state_matrix_array2_t = typename BASE::state_matrix_array2_t;
  using state_matrix_array3_t = typename BASE::state_matrix_array3_t;
  using input_matrix_t = typename BASE::input_matrix_t;
  using input_matrix_array_t = typename BASE::input_matrix_array_t;
  using input_matrix_array2_t = typename BASE::input_matrix_array2_t;
  using input_matrix_array3_t = typename BASE::input_matrix_array3_t;
  using state_input_matrix_t = typename BASE::state_input_matrix_t;
  using state_input_matrix_array_t = typename BASE::state_input_matrix_array_t;
  using state_input_matrix_array2_t = typename BASE::state_input_matrix_array2_t;
  using state_input_matrix_array3_t = typename BASE::state_input_matrix_array3_t;
  using constraint1_vector_t = typename BASE::constraint1_vector_t;
  using constraint1_vector_array_t = typename BASE::constraint1_vector_array_t;
  using constraint1_vector_array2_t = typename BASE::constraint1_vector_array2_t;
  using constraint1_state_matrix_t = typename BASE::constraint1_state_matrix_t;
  using constraint1_state_matrix_array_t = typename BASE::constraint1_state_matrix_array_t;
  using constraint1_state_matrix_array2_t = typename BASE::constraint1_state_matrix_array2_t;
  using constraint1_input_matrix_t = typename BASE::constraint1_input_matrix_t;
  using constraint1_input_matrix_array_t = typename BASE::constraint1_input_matrix_array_t;
  using constraint1_input_matrix_array2_t = typename BASE::constraint1_input_matrix_array2_t;
  using input_constraint1_matrix_t = typename BASE::input_constraint1_matrix_t;
  using input_constraint1_matrix_array_t = typename BASE::input_constraint1_matrix_array_t;
  using input_constraint1_matrix_array2_t = typename BASE::input_constraint1_matrix_array2_t;
  using constraint2_vector_t = typename BASE::constraint2_vector_t;
  using constraint2_vector_array_t = typename BASE::constraint2_vector_array_t;
  using constraint2_vector_array2_t = typename BASE::constraint2_vector_array2_t;
  using constraint2_state_matrix_t = typename BASE::constraint2_state_matrix_t;
  using constraint2_state_matrix_array_t = typename BASE::constraint2_state_matrix_array_t;
  using constraint2_state_matrix_array2_t = typename BASE::constraint2_state_matrix_array2_t;
  using dynamic_vector_t = typename BASE::dynamic_vector_t;
  using dynamic_matrix_t = typename BASE::dynamic_matrix_t;
  using dynamic_vector_array_t = typename BASE::dynamic_vector_array_t;
  using dynamic_matrix_array2_t = typename BASE::dynamic_matrix_array2_t;
  using dynamic_input_matrix_t = typename BASE::dynamic_input_matrix_t;

  using controller_ptr_array_t = typename BASE::controller_ptr_array_t;
  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using linear_controller_array_t = typename linear_controller_t::array_t;

  using event_handler_t = SystemEventHandler<STATE_DIM>;
  using controlled_system_base_t = ControlledSystemBase<STATE_DIM, INPUT_DIM>;
  using derivatives_base_t = DerivativesBase<STATE_DIM, INPUT_DIM>;
  using constraint_base_t = ConstraintBase<STATE_DIM, INPUT_DIM>;
  using cost_function_base_t = CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using operating_trajectories_base_t = SystemOperatingTrajectoriesBase<STATE_DIM, INPUT_DIM>;
  using penalty_base_t = PenaltyBase<STATE_DIM, INPUT_DIM>;

  using rollout_base_t = RolloutBase<STATE_DIM, INPUT_DIM>;
  using time_triggered_rollout_t = TimeTriggeredRollout<STATE_DIM, INPUT_DIM>;
  using linear_quadratic_approximator_t = LinearQuadraticApproximator<STATE_DIM, INPUT_DIM>;
  using operating_trajectorie_rollout_t = OperatingTrajectoriesRollout<STATE_DIM, INPUT_DIM>;

  using cost_desired_trajectories_t = typename BASE::cost_desired_trajectories_t;

  using logic_rules_machine_t = HybridLogicRulesMachine;
  using logic_rules_machine_ptr_t = typename logic_rules_machine_t::Ptr;

  /**
   * Default constructor.
   */
  DDP_BASE() = default;

  /**
   * Constructor
   *
   * @param [in] systemDynamicsPtr: The system dynamics which possibly includes some subsystems.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization.
   * @param [in] ddpSettings: Structure containing the settings for the DDP algorithm.
   * @param [in] rolloutSettings: Structure containing the settings for the rollout.
   * @param [in] logicRulesPtr: The logic rules used for implementing mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation. If it is not
   * defined, we will use the terminal cost function defined in costFunctionPtr.
   */
  DDP_BASE(const controlled_system_base_t* systemDynamicsPtr, const derivatives_base_t* systemDerivativesPtr,
           const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
           const operating_trajectories_base_t* operatingTrajectoriesPtr, const DDP_Settings& ddpSettings,
           const Rollout_Settings& rolloutSettings, const cost_function_base_t* heuristicsFunctionPtr, const char* algorithmName,
           std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr);

  /**
   * Destructor.
   */
  virtual ~DDP_BASE();

  /**
   * Resets the class to its state after construction.
   */
  void reset() override;

  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: Time partitioning
   * @param [in] controllersStock: Array of control policies.
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [in] threadId: Working thread (default is 0).
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                             const scalar_array_t& partitioningTimes, linear_controller_array_t& controllersStock,
                             scalar_array2_t& timeTrajectoriesStock, size_array2_t& eventsPastTheEndIndecesStock,
                             state_vector_array2_t& stateTrajectoriesStock, input_vector_array2_t& inputTrajectoriesStock,
                             size_t threadId = 0);

  /**
   * The class for performing rollout. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime] and only return the final state.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: Time partitioning
   * @param [in] controllersStock: Array of control policies.
   * @param [out] finalState: Final state.
   * @param [out] finalInput: Final control input.
   * @param [out] finalActiveSubsystemIndex: The final active subsystem.
   * @param [in] threadId: Working thread (default is 0).
   */
  void rolloutFinalState(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
                         const linear_controller_array_t& controllersStock, state_vector_t& finalState, input_vector_t& finalInput,
                         size_t& finalActivePartition, size_t threadId = 0);

  /**
   * Calculates a rollout constraints. It uses the given rollout trajectories and calculate the constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] nc1TrajectoriesStock: Array of trajectories containing the number of the active state-input constraints.
   * @param [out] EvTrajectoryStock: Array of trajectories containing the value of the state-input constraints (if the
   * rollout is constrained the value is always zero otherwise it is nonzero).
   * @param [out] nc2TrajectoriesStock: Array of trajectories containing the number of the active state-only constraints.
   * @param [out] HvTrajectoryStock: Array of trajectories containing the value of the state-only constraints.
   * @param [out] nc2FinalStock: Array containing the number of the active final state-only constraints.
   * @param [out] HvFinalStock: Array containing the value of the final state-only constraints.
   * @param [in] threadId: Working thread (default is 0).
   */
  void calculateRolloutConstraints(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& eventsPastTheEndIndecesStock,
                                   const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                                   size_array2_t& nc1TrajectoriesStock, constraint1_vector_array2_t& EvTrajectoryStock,
                                   size_array2_t& nc2TrajectoriesStock, constraint2_vector_array2_t& HvTrajectoryStock,
                                   size_array2_t& ncIneqTrajectoriesStock, scalar_array3_t& hTrajectoryStock, size_array2_t& nc2FinalStock,
                                   constraint2_vector_array2_t& HvFinalStock, size_t threadId = 0);

  /**
   * Calculates cost of a rollout.
   *
   * @param [in] threadId: Working thread.
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
   * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
   * @param [out] totalCost: The total cost of the rollout.
   * @param [in] threadId: Working thread (default is 0).
   */
  void calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& eventsPastTheEndIndecesStock,
                            const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                            scalar_t& totalCost, size_t threadId = 0);

  /**
   * Calculates the cost function plus penalty for state-only constraints of a rollout.
   *
   * @param [in] threadId: Working thread.
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
   * @param [in] eventsPastTheEndIndecesStock: Array of indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
   * @param [in] constraint2ISE: Type-2 constraint's ISE (Integral Squared Error).
   * @param [in] nc2FinalStock: Array containing the number of the active final state-only constraints.
   * @param [in] HvFinalStock: Array containing the value of the final state-only constraints.
   * @param [out] totalCost: The total cost plus state-only constraints penalty.
   * @param [in] threadId: Working thread (default is 0).
   */
  void calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& eventsPastTheEndIndecesStock,
                            const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                            scalar_t constraint2ISE, scalar_t inequalityConstraintPenalty, const size_array2_t& nc2FinalStock,
                            const constraint2_vector_array2_t& HvFinalStock, scalar_t& totalCost, size_t threadId = 0);

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the nominal
   * state and control trajectories. This method updates the following variables:
   * 	- linearized system model and constraints
   * 	- \f$ dxdt = A_m(t)x + B_m(t)u \f$.
   * 	- s.t. \f$ C_m(t)x + D_m(t)u + E_v(t) = 0 \f$ \\
   * 	-      \f$ F_m(t)x + H_v(t) = 0 \f$ .
   * 	- AmTrajectoryStock_: \f$ A_m\f$  matrix.
   * 	- BmTrajectoryStock_: \f$ B_m\f$  matrix.
   * 	- CmTrajectoryStock_: \f$ C_m\f$ matrix.
   * 	- DmTrajectoryStock_: \f$ D_m\f$ matrix.
   * 	- EvTrajectoryStock_: \f$ E_v\f$ vector.
   * 	- FmTrajectoryStock_: \f$ F_m\f$ vector.
   * 	- HvTrajectoryStock_: \f$ H_v\f$ vector.
   *
   * 	- quadratized intermediate cost function
   * 	- intermediate cost: \f$ q(t) + 0.5 xQ_m(t)x + x'Q_v(t) + u'P_m(t)x + 0.5u'R_m(t)u + u'R_v(t) \f$
   * 	- qTrajectoryStock_:  \f$ q\f$
   * 	- QvTrajectoryStock_: \f$ Q_v\f$ vector.
   * 	- QmTrajectoryStock_:\f$  Q_m\f$ matrix.
   * 	- PmTrajectoryStock_: \f$ P_m\f$ matrix.
   * 	- RvTrajectoryStock_: \f$ R_v\f$ vector.
   * 	- RmTrajectoryStock_: \f$ R_m\f$ matrix.
   *
   * 	- as well as the constrained coefficients of
   * 		- linearized system model
   * 		- quadratized intermediate cost function
   * 		- quadratized final cost
   *
   */
  virtual void approximateOptimalControlProblem();

  /**
   * Calculates the controller. This method uses the following variables:
   * - constrained, linearized model
   * - constrained, quadratized cost
   *
   * The method modifies:
   * - nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
   * 					improves the constraints as well as the increment to the feed-forward control input.
   */
  virtual void calculateController() = 0;

  /**
   * Line search on the feedforward parts of the controller. It uses the following approach for line search:
   * The constraint TYPE-1 correction term is directly added through a user defined stepSize (defined in settings_.constraintStepSize_).
   * But the cost minimization term is optimized through a line-search strategy defined in ILQR settings.
   *
   * @param [in] computeISEs: Whether lineSearch needs to calculate ISEs indices for type_1 and type-2 constraints.
   */
  virtual void lineSearch(bool computeISEs) = 0;

  /**
   * Solves Riccati equations for all the partitions.
   *
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   *
   * @return average time step
   */
  virtual scalar_t solveSequentialRiccatiEquations(const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
                                                   const eigen_scalar_t& sFinal) = 0;

  /**
   * Adjust the nominal controller based on the last changes in the logic rules.
   *
   * @param [in] newEventTimes: The new event times.
   * @param [in] controllerEventTimes: The control policy stock's event times.
   */
  void adjustController(const scalar_array_t& newEventTimes, const scalar_array_t& controllerEventTimes);

  scalar_t getValueFuntion(scalar_t time, const state_vector_t& state) const override;

  void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx) const override;

  virtual void getLinearFeedbackGain(scalar_t time, input_state_matrix_t& K) const override;

  /**
   * Upon activation in the multi-thread DDP class (DDP_MT), the parallelization of the backward pass takes
   * place from the the first iteration which normally become effective after the first iteration.
   *
   * @param [in] flag: If set true, the parallel Riccati solver will be used from the first iteration.
   */
  void useParallelRiccatiSolverFromInitItr(bool flag);

  void blockwiseMovingHorizon(bool flag) override;

  void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override;

  size_t getNumIterations() const override;

  void getIterationsLog(eigen_scalar_array_t& iterationCost, eigen_scalar_array_t& iterationISE1,
                        eigen_scalar_array_t& iterationISE2) const override;

  void getIterationsLogPtr(const eigen_scalar_array_t*& iterationCostPtr, const eigen_scalar_array_t*& iterationISE1Ptr,
                           const eigen_scalar_array_t*& iterationISE2Ptr) const override;

  /**
   * Write access to ddp settings
   */
  DDP_Settings& ddpSettings();

  const controller_ptr_array_t& getController() const override;

  void getControllerPtr(const controller_ptr_array_t*& controllersPtrStock) const override;

  const scalar_array2_t& getNominalTimeTrajectories() const override;

  const state_vector_array2_t& getNominalStateTrajectories() const override;

  const input_vector_array2_t& getNominalInputTrajectories() const override;

  void getNominalTrajectoriesPtr(const scalar_array2_t*& nominalTimeTrajectoriesStockPtr,
                                 const state_vector_array2_t*& nominalStateTrajectoriesStockPtr,
                                 const input_vector_array2_t*& nominalInputTrajectoriesStockPtr) const override;

  void swapNominalTrajectories(scalar_array2_t& nominalTimeTrajectoriesStock, state_vector_array2_t& nominalStateTrajectoriesStock,
                               input_vector_array2_t& nominalInputTrajectoriesStock) override;

  scalar_t getFinalTime() const override;

  const scalar_array_t& getPartitioningTimes() const override;

  void rewindOptimizer(size_t firstIndex) override;

  const unsigned long long int& getRewindCounter() const override;

  /**
   * Runs the initialization method for DDP.
   *
   */
  virtual void runInit();

  /**
   * Runs a single iteration of DDP.
   *
   */
  virtual void runIteration();

  /**
   * Runs the exit method DDP.
   */
  virtual void runExit();

  /**
   * The main routine of DDP which runs DDP for a given initial state, initial time, and final time. In order
   * to retrieve the initial nominal trajectories in the forward pass, DDP will use the given operatingTrajectories
   * in the constructor.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   */
  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  /**
   * The main routine of DDP which runs DDP for a given initial state, initial time, and final time. In order
   * to retrieve the initial nominal trajectories in the forward pass, DDP will use the provided control policy.
   * If you want to use the control policy which was designed by the previous call of the "run" routine, you
   * should pass DDP_BASE::INTERNAL_CONTROLLER().
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The time partitioning.
   * @param [in] controllersPtrStock: Array of pointers to the initial control policies. If you want to use the control policy
   * which was designed by the previous call of the "run" routine, you should pass an empty array.
   * In the this case, two scenarios are possible: either the internal controller is already set (such as the MPC case
   * where the warm starting option is set true) or the internal controller is empty in which instead of performing
   * a rollout the operating trajectories will be used.
   */
  void run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
           const controller_ptr_array_t& controllersPtrStock) override;

 protected:
  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(size_t numPartitions);

  /**
   * Computes the linearized dynamics for a particular time partition
   *
   * @param [in] partitionIndex: Time partition index
   */
  virtual void approximatePartitionLQ(size_t partitionIndex) = 0;

  /**
   * Computes the controller for a particular time partition
   *
   * @param partitionIndex: Time partition index
   */
  virtual void calculatePartitionController(size_t partitionIndex) = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeIndex: Time index in the partition.
   */
  virtual void approximateLQWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) = 0;

  /**
   * Calculates the constraint trajectories over the given trajectories.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   * @param [out] nc1Trajectory: Trajectory containing number of active type-1 constraints.
   * @param [out] EvTrajectory: Type-1 constraints trajectory.
   * @param [out] nc2Trajectory: Trajectory containing number of active type-2 constraints.
   * @param [out] HvTrajectory: Type-2 constraints trajectory.
   * @param [out] nc2Finals: Number of active final type-2 constraints.
   * @param [out] HvFinals: Final type-2 constraints.
   */
  virtual void calculateConstraintsWorker(size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory,
                                          const size_array_t& eventsPastTheEndIndeces, const state_vector_array_t& stateTrajectory,
                                          const input_vector_array_t& inputTrajectory, size_array_t& nc1Trajectory,
                                          constraint1_vector_array_t& EvTrajectory, size_array_t& nc2Trajectory,
                                          constraint2_vector_array_t& HvTrajectory, size_array_t& ncIneqTrajectory,
                                          scalar_array2_t& hTrajectory, size_array_t& nc2Finals, constraint2_vector_array_t& HvFinals);

  /**
   * Calculates the total cost for the given trajectories.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   * @param [out] totalCost: The total cost.
   */
  virtual void calculateCostWorker(size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory,
                                   const size_array_t& eventsPastTheEndIndeces, const state_vector_array_t& stateTrajectory,
                                   const input_vector_array_t& inputTrajectory, scalar_t& totalCost);

  /**
   * Calculates an LQ approximate of the unconstrained optimal control problem at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] i: Time partition index.
   * @param [in] k: Time index in the partition.
   */
  virtual void approximateUnconstrainedLQWorker(size_t workerIndex, size_t i, size_t k);

  /**
   * Calculates an LQ approximate of the event times process.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] i: Time partition index.
   * @param [in] k: Time index in the partition.
   * @param [in] stateConstraintPenalty: State-only constraint penalty.
   */
  virtual void approximateEventsLQWorker(size_t workerIndex, size_t i, size_t k, scalar_t stateConstraintPenalty);

  /**
   * Calculates controller at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index
   * @param [in] timeIndex: Time index in the partition
   */
  virtual void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) = 0;

  /**
   * Performs one rollout while the input correction for the type-1 constraint is considered.
   *
   * @param [in] computeISEs: Whether needs to calculate ISEs indices for type_1 and type-2 constraints.
   */
  virtual void lineSearchBase(bool computeISEs);

  /**
   * Line search with a specific learning rate.
   *
   * @param workerIndex
   * @param learningRate
   * @param lsTotalCost
   * @param lsConstraint1ISE
   * @param lsConstraint1MaxNorm
   * @param lsConstraint2ISE
   * @param lsConstraint2MaxNorm
   * @param lsInequalityConstraintPenalty
   * @param lsInequalityConstraintISE
   * @param lsControllersStock
   * @param lsTimeTrajectoriesStock
   * @param lsEventsPastTheEndIndecesStock
   * @param lsStateTrajectoriesStock
   * @param lsInputTrajectoriesStock
   */
  virtual void lineSearchWorker(size_t workerIndex, scalar_t learningRate, scalar_t& lsTotalCost, scalar_t& lsConstraint1ISE,
                                scalar_t& lsConstraint1MaxNorm, scalar_t& lsConstraint2ISE, scalar_t& lsConstraint2MaxNorm,
                                scalar_t& lsInequalityConstraintPenalty, scalar_t& lsInequalityConstraintISE,
                                linear_controller_array_t& lsControllersStock, scalar_array2_t& lsTimeTrajectoriesStock,
                                size_array2_t& lsEventsPastTheEndIndecesStock, state_vector_array2_t& lsStateTrajectoriesStock,
                                input_vector_array2_t& lsInputTrajectoriesStock);

  /**
   * compute the merit function for given rollout
   *
   * @param [in] timeTrajectoriesStock: simulation time trajectory
   * @param [in] nc1TrajectoriesStock: rollout's number of active constraints in each time step
   * @param [in] EvTrajectoryStock: rollout's constraints value
   * @param [in] lagrangeTrajectoriesStock: constraint Lagrange multiplier for the given rollout
   * @param [in] totalCost: the total cost of the trajectory
   * @param [out] meritFunctionValue: the total merit function value of the trajectory
   * @param [out] constraintISE: Integral of Square Error (ISE)
   */
  void calculateMeritFunction(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& nc1TrajectoriesStock,
                              const constraint1_vector_array2_t& EvTrajectoryStock,
                              const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock, scalar_t totalCost,
                              scalar_t& meritFunctionValue, scalar_t& constraintISE);

  /**
   * Calculates state-input constraints ISE (Integral of Square Error). It also return the maximum norm of the constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp.
   * @param [in] nc1TrajectoriesStock: Array of trajectories containing the number of the active state-input constraints.
   * @param [in] EvTrajectoriesStock: Array of trajectories containing the value of the state-input constraints.
   * @param [out] constraintISE: The state-input constraints ISE.
   * @return maximum norm of the constraints.
   */
  scalar_t calculateConstraintISE(const scalar_array2_t& timeTrajectoriesStock,
                                  const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
                                  const constraint1_vector_array2_t& EvTrajectoriesStock, scalar_t& constraintISE);

  /**
   * Calculate integrated penalty from inequality constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp.
   * @param [in] ncIneqTrajectoriesStock: Array of trajectories containing the number of inequalityConstraints
   * @param [in] hTrajectoriesStock: Array of trajectories containing the value of the inequality constraints.
   * @param [in] penaltyPtrStock: Array of penalty function pointers.
   * @return constraintPenalty: The inequality constraints penalty.
   */
  scalar_t calculateInequalityConstraintPenalty(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& ncIneqTrajectoriesStock,
                                                const scalar_array3_t& hTrajectoriesStock, scalar_t& inequalityISE, size_t workerIndex = 0);
  /**
   * Truncates the internal array of the control policies based on the initTime.
   *
   * @param [in] partitioningTimes: Switching times.
   * @param [in] initTime: Initial time.
   * @param [out] controllersStock: Truncated array of the control policies.
   * @param [out] initActivePartition: Initial active subsystems.
   * @param [out] deletedcontrollersStock: The deleted part of the control policies.
   */
  void truncateController(const scalar_array_t& partitioningTimes, double initTime, linear_controller_array_t& controllersStock,
                          size_t& initActivePartition, linear_controller_array_t& deletedcontrollersStock);

  /**
   * Calculates max feedforward update norm and max type-1 error update norm.
   *
   * @param maxDeltaUffNorm: max feedforward update norm.
   * @param maxDeltaUeeNorm: max type-1 error update norm.
   */
  void calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm);

  /**
   * Updates pointers in nominalControllerPtrStock from memory location of nominalControllersStock_ members.
   */
  void updateNominalControllerPtrStock();

  /**
   * Display rollout info and scores.
   */
  void printRolloutInfo();

  // Variables
  DDP_Settings ddpSettings_;
  Rollout_Settings rolloutSettings_;

  std::string algorithmName_;

  unsigned long long int rewindCounter_;

  bool useParallelRiccatiSolverFromInitItr_ = false;
  // If true the final time of the MPC will increase by a time partition instead of common gradual increase.
  bool blockwiseMovingHorizon_ = false;

  scalar_t initTime_;
  scalar_t finalTime_;
  state_vector_t initState_;

  size_t initActivePartition_;
  size_t finalActivePartition_;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  const scalar_array2_t* desiredTimeTrajectoryStockPtr_;
  const state_vector_array2_t* desiredStateTrajectoryStockPtr_;
  const input_vector_array2_t* desiredInputTrajectoryStockPtr_;

  scalar_t learningRateStar_ = 1.0;  // The optimal learning rate.
  scalar_t maxLearningRate_ = 1.0;   // The maximum permitted learning rate (settings_.maxLearningRateSLQ_).
  scalar_t constraintStepSize_ = 1.0;

  // It is true if an initial controller is not provided for a partition which causes that the first
  // iteration of SLQ to design an initial controller (LQR). In this case:
  // 1) The feedforward component and the type-1 constraint input are set to zero
  // 2) Final cost will be ignored
  std::vector<bool> initialControllerDesignStock_;

  // trajectory spreading
  TrajectorySpreadingController<STATE_DIM, INPUT_DIM> trajectorySpreadingController_;

  size_t iteration_;
  eigen_scalar_array_t iterationCost_;
  eigen_scalar_array_t iterationISE1_;
  eigen_scalar_array_t iterationISE2_;

  scalar_t nominalTotalCost_;
  scalar_t nominalConstraint1ISE_;
  scalar_t nominalConstraint1MaxNorm_;
  scalar_t nominalConstraint2ISE_;
  scalar_t nominalConstraint2MaxNorm_;
  scalar_t nominalInequalityConstraintPenalty_;
  scalar_t nominalInequalityConstraintISE_;

  // Forward pass and backward pass average time step
  scalar_t avgTimeStepFP_;
  scalar_t avgTimeStepBP_;

  std::vector<typename rollout_base_t::Ptr> dynamicsForwardRolloutPtrStock_;
  std::vector<typename rollout_base_t::Ptr> operatingTrajectoriesRolloutPtrStock_;
  std::vector<std::unique_ptr<linear_quadratic_approximator_t>> linearQuadraticApproximatorPtrStock_;
  std::vector<typename cost_function_base_t::Ptr> heuristicsFunctionsPtrStock_;
  std::vector<typename operating_trajectories_base_t::Ptr> operatingTrajectoriesPtrStock_;
  std::vector<std::shared_ptr<penalty_base_t>> penaltyPtrStock_;

  linear_controller_array_t nominalControllersStock_;
  controller_ptr_array_t nominalControllerPtrStock_;

  std::vector<scalar_array_t> nominalTimeTrajectoriesStock_;
  std::vector<size_array_t> nominalEventsPastTheEndIndecesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;

  // Used for catching the nominal trajectories for which the LQ problem is constructed and solved before terminating run()
  scalar_array2_t nominalPrevTimeTrajectoriesStock_;
  size_array2_t nominalPrevEventsPastTheEndIndecesStock_;
  state_vector_array2_t nominalPrevStateTrajectoriesStock_;
  input_vector_array2_t nominalPrevInputTrajectoriesStock_;

  bool lsComputeISEs_;                                // whether lineSearch routine needs to calculate ISEs
  linear_controller_array_t initLScontrollersStock_;  // needed for lineSearch

  linear_controller_array_t deletedcontrollersStock_;  // needed for concatenating the new controller to the old one

  std::vector<EigenLinearInterpolation<state_vector_t>> nominalStateFunc_;
  std::vector<EigenLinearInterpolation<input_vector_t>> nominalInputFunc_;

  state_matrix_array2_t AmTrajectoryStock_;
  state_input_matrix_array2_t BmTrajectoryStock_;

  size_array2_t nc1TrajectoriesStock_;  // nc1: Number of the Type-1  active constraints
  constraint1_vector_array2_t EvTrajectoryStock_;
  constraint1_state_matrix_array2_t CmTrajectoryStock_;
  constraint1_input_matrix_array2_t DmTrajectoryStock_;

  size_array2_t nc2TrajectoriesStock_;  // nc2: Number of the Type-2 active constraints
  constraint2_vector_array2_t HvTrajectoryStock_;
  constraint2_state_matrix_array2_t FmTrajectoryStock_;
  size_array2_t nc2FinalStock_;
  constraint2_vector_array2_t HvFinalStock_;
  constraint2_state_matrix_array2_t FmFinalStock_;

  size_array2_t ncIneqTrajectoriesStock_;  // ncIneq: Number of inequality constraints
  scalar_array3_t hTrajectoryStock_;
  state_vector_array3_t dhdxTrajectoryStock_;
  state_matrix_array3_t ddhdxdxTrajectoryStock_;
  input_vector_array3_t dhduTrajectoryStock_;
  input_matrix_array3_t ddhduduTrajectoryStock_;
  input_state_matrix_array3_t ddhdudxTrajectoryStock_;

  eigen_scalar_array2_t qTrajectoryStock_;
  state_vector_array2_t QvTrajectoryStock_;
  state_matrix_array2_t QmTrajectoryStock_;
  input_vector_array2_t RvTrajectoryStock_;
  input_matrix_array2_t RmTrajectoryStock_;
  input_state_matrix_array2_t PmTrajectoryStock_;

  eigen_scalar_array2_t qFinalStock_;
  state_vector_array2_t QvFinalStock_;
  state_matrix_array2_t QmFinalStock_;

  // Riccati solution coefficients
  scalar_array2_t SsTimeTrajectoryStock_;
  scalar_array2_t SsNormalizedTimeTrajectoryStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  eigen_scalar_array2_t sTrajectoryStock_;
  state_vector_array2_t SvTrajectoryStock_;
  state_vector_array2_t SveTrajectoryStock_;
  state_matrix_array2_t SmTrajectoryStock_;

  eigen_scalar_array_t sFinalStock_;
  state_vector_array_t SvFinalStock_;
  state_vector_array_t SveFinalStock_;
  state_matrix_array_t SmFinalStock_;
  state_vector_array_t xFinalStock_;

  eigen_scalar_t sHeuristics_;
  state_vector_t SvHeuristics_;
  state_matrix_t SmHeuristics_;

#ifdef BENCHMARK
  // Benchmarking
  size_t BENCHMARK_nIterationsLQ_ = 0;
  size_t BENCHMARK_nIterationsBP_ = 0;
  size_t BENCHMARK_nIterationsFP_ = 0;
  scalar_t BENCHMARK_tAvgLQ_ = 0.0;
  scalar_t BENCHMARK_tAvgBP_ = 0.0;
  scalar_t BENCHMARK_tAvgFP_ = 0.0;
  std::chrono::time_point<std::chrono::steady_clock> BENCHMARK_start_;
  std::chrono::time_point<std::chrono::steady_clock> BENCHMARK_end_;
  std::chrono::duration<double> BENCHMARK_diff_;
#endif
};

}  // namespace ocs2

#include "implementation/DDP_BASE.h"

#endif /* DDP_BASE_OCS2_H_ */
