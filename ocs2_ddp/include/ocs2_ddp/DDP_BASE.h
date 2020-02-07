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

#pragma once

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/TrajectorySpreadingControllerAdjustment.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/misc/ThreadPool.h>
#include <ocs2_core/model_data/ModelDataBase.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/rollout/OperatingTrajectoriesRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_ddp/DDP_Settings.h"
#include "ocs2_ddp/riccati_equations/RiccatiModificationBase.h"
#include "ocs2_ddp/riccati_equations/RiccatiModificationInterpolation.h"

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
  using BASE = Solver_BASE<STATE_DIM, INPUT_DIM>;

  using typename BASE::constraint1_input_matrix_array2_t;
  using typename BASE::constraint1_input_matrix_array_t;
  using typename BASE::constraint1_input_matrix_t;
  using typename BASE::constraint1_state_matrix_array2_t;
  using typename BASE::constraint1_state_matrix_array_t;
  using typename BASE::constraint1_state_matrix_t;
  using typename BASE::constraint1_vector_array2_t;
  using typename BASE::constraint1_vector_array_t;
  using typename BASE::constraint1_vector_t;
  using typename BASE::constraint2_state_matrix_array2_t;
  using typename BASE::constraint2_state_matrix_array_t;
  using typename BASE::constraint2_state_matrix_t;
  using typename BASE::constraint2_vector_array2_t;
  using typename BASE::constraint2_vector_array_t;
  using typename BASE::constraint2_vector_t;
  using typename BASE::DIMENSIONS;
  using typename BASE::dynamic_input_matrix_t;
  using typename BASE::dynamic_matrix_array2_t;
  using typename BASE::dynamic_matrix_array_t;
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_array2_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::input_dynamic_matrix_array2_t;
  using typename BASE::input_dynamic_matrix_array_t;
  using typename BASE::input_dynamic_matrix_t;
  using typename BASE::input_matrix_array2_t;
  using typename BASE::input_matrix_array3_t;
  using typename BASE::input_matrix_array_t;
  using typename BASE::input_matrix_t;
  using typename BASE::input_state_matrix_array2_t;
  using typename BASE::input_state_matrix_array3_t;
  using typename BASE::input_state_matrix_array_t;
  using typename BASE::input_state_matrix_t;
  using typename BASE::input_vector_array2_t;
  using typename BASE::input_vector_array3_t;
  using typename BASE::input_vector_array_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_array2_t;
  using typename BASE::scalar_array3_t;
  using typename BASE::scalar_array_t;
  using typename BASE::scalar_t;
  using typename BASE::size_array2_t;
  using typename BASE::size_array_t;
  using typename BASE::state_input_matrix_array2_t;
  using typename BASE::state_input_matrix_array3_t;
  using typename BASE::state_input_matrix_array_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_array2_t;
  using typename BASE::state_matrix_array3_t;
  using typename BASE::state_matrix_array_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_array2_t;
  using typename BASE::state_vector_array3_t;
  using typename BASE::state_vector_array_t;
  using typename BASE::state_vector_t;

  using typename BASE::controller_array_t;
  using typename BASE::controller_const_ptr_array_t;
  using typename BASE::controller_ptr_array_t;
  using typename BASE::controller_t;
  using typename BASE::feedforward_controller_t;
  using typename BASE::primal_solution_t;

  using linear_controller_t = LinearController<STATE_DIM, INPUT_DIM>;
  using linear_controller_array_t = typename linear_controller_t::array_t;
  using linear_controller_ptr_array_t = std::vector<linear_controller_t*>;

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

  using logic_rules_machine_t = HybridLogicRulesMachine;
  using logic_rules_machine_ptr_t = typename logic_rules_machine_t::Ptr;

  /**
   * Default constructor.
   */
  DDP_BASE() = default;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDerivativesPtr: The system dynamics derivatives for
   * subsystems of the system.
   * @param [in] systemConstraintsPtr: The system constraint function and its
   * derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and terminal
   * costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system
   * which will be used for initialization.
   * @param [in] ddpSettings: Structure containing the settings for the DDP
   * algorithm.
   * @param [in] logicRulesPtr: The logic rules used for implementing
   * mixed-logic dynamical systems.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite
   * time optimal control formulation. If it is not defined, we will use the
   * terminal cost function defined in costFunctionPtr.
   */
  DDP_BASE(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
           const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
           const DDP_Settings& ddpSettings, const cost_function_base_t* heuristicsFunctionPtr, const char* algorithmName,
           std::shared_ptr<HybridLogicRules> logicRulesPtr = nullptr);

  /**
   * Destructor.
   */
  virtual ~DDP_BASE() override;

  /**
   * Resets the class to its state after construction.
   */
  void reset() override;

  /**
   * Forward integrate the system dynamics with given controller. It uses the
   * given control policies and initial state, to integrate the system dynamics
   * in time period [initTime, finalTime].
   *
   * @param [in] controllersStock: Array of control policies.
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] postEventIndicesStock: Array of the post-event indices.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [in] threadId: Working thread (default is 0).
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(linear_controller_array_t& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                             size_array2_t& postEventIndicesStock, state_vector_array2_t& stateTrajectoriesStock,
                             input_vector_array2_t& inputTrajectoriesStock, ModelDataBase::array2_t& modelDataTrajectoriesStock,
                             size_t threadId = 0);

  /**
   * Calculates a rollout constraints. It uses the given rollout trajectories
   * and calculate the constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the
   * output time trajectory stamp.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the
   * output state trajectory.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the
   * output control input trajectory.
   * @param [out] nc1TrajectoriesStock: Array of trajectories containing the
   * number of the active state-input constraints.
   * @param [out] EvTrajectoryStock: Array of trajectories containing the value
   * of the state-input constraints (if the rollout is constrained the value is
   * always zero otherwise it is nonzero).
   * @param [out] nc2TrajectoriesStock: Array of trajectories containing the
   * number of the active state-only constraints.
   * @param [out] HvTrajectoryStock: Array of trajectories containing the value
   * of the state-only constraints.
   * @param [out] nc2FinalStock: Array containing the number of the active final
   * state-only constraints.
   * @param [out] HvFinalStock: Array containing the value of the final
   * state-only constraints.
   * @param [in] threadId: Working thread (default is 0).
   */
  void calculateRolloutConstraints(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                   const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                                   size_array2_t& nc1TrajectoriesStock, constraint1_vector_array2_t& EvTrajectoryStock,
                                   size_array2_t& nc2TrajectoriesStock, constraint2_vector_array2_t& HvTrajectoryStock,
                                   size_array2_t& ncIneqTrajectoriesStock, scalar_array3_t& hTrajectoryStock, size_array2_t& nc2FinalStock,
                                   constraint2_vector_array2_t& HvFinalStock, size_t threadId = 0);

  /**
   * Calculates constraints ISE (Integral of Square Error).
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] stateInputEqConstraintISE: The state-input equality constraints ISE.
   * @param [out] stateEqConstraintISE: The state-only equality constraints ISE.
   * @param [out] stateEqFinalConstraintISE: The final state equality constraints ISE.
   * @param [out] inequalityConstraintISE: The inequality constraints ISE.
   * @param [out] inequalityConstraintPenalty: The inequality constraints penalty.
   * @return maximum norm of the constraints.
   */
  void calculateRolloutConstraintsISE(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                      const state_vector_array2_t& stateTrajectoriesStock,
                                      const input_vector_array2_t& inputTrajectoriesStock, scalar_t& stateInputEqConstraintISE,
                                      scalar_t& stateEqConstraintISE, scalar_t& stateEqFinalConstraintISE,
                                      scalar_t& inequalityConstraintISE, scalar_t& inequalityConstraintPenalty, size_t workerIndex = 0);

  /**
   * Calculates cost of a rollout.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
   * @param [in] threadId: Working thread.
   * @return The total cost of the rollout.
   */
  scalar_t calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                                size_t threadId);

  /**
   * Calculates cost of a merit.
   *
   * @param [in] cost: The rollout cost.
   * @param [in] stateInputEqConstraintISE: The state-input equality constraints ISE.
   * @param [in] stateEqConstraintISE: The state-only equality constraints ISE.
   * @param [in] stateEqFinalConstraintISE: The final state equality constraints ISE.
   * @param [in] inequalityConstraintPenalty: The inequality constraints penalty.
   * @return Merit value.
   */
  inline scalar_t calculateRolloutMerit(const scalar_t& cost, const scalar_t& stateInputEqConstraintISE,
                                        const scalar_t& stateEqConstraintISE, const scalar_t& stateEqFinalConstraintISE,
                                        const scalar_t& inequalityConstraintPenalty) const;

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the
   * nominal state and control trajectories. This method updates the following
   * variables:
   * 	- linearized system model and constraints
   * 	- quadratized cost function
   * 	- as well as the constrained coefficients of
   * 		- linearized system model
   * 		- quadratized intermediate cost function
   * 		- quadratized final cost
   */
  void approximateOptimalControlProblem();

  /**
   * Calculates the controller. This method uses the following variables:
   * - constrained, linearized model
   * - constrained, quadratized cost
   *
   * The method modifies:
   * - nominalControllersStock_: the controller that stabilizes the system
   * around the new nominal trajectory and improves the constraints as well as
   * the increment to the feed-forward control input.
   */
  virtual void calculateController();

  /**
   * Calculates the integral of the squared (IS) norm of the controller update.
   *
   * @param [in] controllersStock: An array of controllers.
   * @retuen The integral of the squared (IS) norm of the controller update.
   */
  scalar_t calculateControllerUpdateIS(const linear_controller_array_t& controllersStock) const;

  /**
   * Levenberg Marquardt strategy.
   */
  void levenbergMarquardt();

  /**
   * Line search on the feedforward parts of the controller. It uses the
   * following approach for line search: The constraint TYPE-1 correction term
   * is directly added through a user defined step length (defined in
   * settings_.constraintStepSize_). But the cost minimization term is optimized
   * through a line-search strategy defined in ILQR settings.
   */
  void lineSearch();

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
                                                   const scalar_t& sFinal) = 0;

  /**
   * The implementation for solving Riccati equations for all the partitions.
   *
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   *
   * @return average time step
   */
  scalar_t solveSequentialRiccatiEquationsImpl(const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const scalar_t& sFinal);

  /**
   * Solves Riccati equations for the partitions assigned to the given thread.
   */
  void riccatiSolverTask();

  /**
   * Solves a set of Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   */
  virtual void riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const dynamic_matrix_t& SmFinal,
                                      const dynamic_vector_t& SvFinal, const scalar_t& sFinal) = 0;

  /**
   * Computes the normalized time for Riccati backward pass.
   *
   * @param [in] timeTrajectory: The time trajectory.
   * @param [in] postEventIndices: The post event indices.
   * @param [out] normalizedTimeTrajectory: The reversed and negated timeTrajectory.
   * @param [out] strategy: The corresponding post event indices of normalizedTimeTrajectory.
   */
  static void computeNormalizedTime(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                    scalar_array_t& normalizedTimeTrajectory, size_array_t& normalizedPostEventIndices);

  /**
   * Adjust the nominal controller based on the last changes in the logic rules.
   *
   * @param [in] newEventTimes: The new event times.
   * @param [in] controllerEventTimes: The control policy stock's event times.
   */
  void adjustController(const scalar_array_t& newEventTimes, const scalar_array_t& controllerEventTimes);

  scalar_t getValueFunction(scalar_t time, const state_vector_t& state) const override;

  void getValueFunctionStateDerivative(scalar_t time, const state_vector_t& state, state_vector_t& Vx) const override;

  /**
   * Upon activation in the multi-thread DDP class (DDP_MT), the parallelization
   * of the backward pass takes place from the the first iteration which
   * normally become effective after the first iteration.
   *
   * @param [in] flag: If set true, the parallel Riccati solver will be used
   * from the first iteration.
   */
  void useParallelRiccatiSolverFromInitItr(bool flag);

  void getPerformanceIndeces(scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) const override;

  size_t getNumIterations() const override;

  void getIterationsLog(scalar_array_t& iterationCost, scalar_array_t& iterationISE1, scalar_array_t& iterationISE2) const override;

  void getStateInputConstraintLagrangian(scalar_t time, const state_vector_t& state, dynamic_vector_t& nu) const override;

  /**
   * Write access to ddp settings
   */
  DDP_Settings& ddpSettings();

  /**
   * Const access to ddp settings
   */
  const DDP_Settings& ddpSettings() const;

  void getPrimalSolution(scalar_t finalTime, primal_solution_t* primalSolutionPtr) const final;

  scalar_t getFinalTime() const override;

  const scalar_array_t& getPartitioningTimes() const override;

  void rewindOptimizer(size_t firstIndex) override;

  const unsigned long long int& getRewindCounter() const override;

  /**
   * Runs the initialization method for DDP.
   */
  void runInit();

  /**
   * Runs a single iteration of DDP.
   */
  void runIteration();

 protected:
  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(size_t numPartitions);

  /**
   * Distributes the sequential tasks (e.g. Riccati solver) in between threads.
   */
  void distributeWork();

  /**
   * Helper to run task multiple times in parallel (blocking)
   *
   * @param [in] taskFunction: task function
   * @param [in] N: number of times to run taskFunction, if N = 1 it is run in the main thread
   */
  void runParallel(std::function<void(void)> taskFunction, size_t N);

  /**
   * Calculates the constraint trajectories over the given trajectories.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index.
   * @param [in] timeTrajectory: The time trajectory stamp.
   * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index
   * of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   * @param [out] nc1Trajectory: Trajectory containing number of active type-1
   * constraints.
   * @param [out] EvTrajectory: Type-1 constraints trajectory.
   * @param [out] nc2Trajectory: Trajectory containing number of active type-2
   * constraints.
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
   * @param [in] eventsPastTheEndIndeces: Indices containing past-the-end index
   * of events trigger.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The control input trajectory.
   * @param [out] totalCost: The total cost.
   */
  virtual void calculateCostWorker(size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory,
                                   const size_array_t& eventsPastTheEndIndeces, const state_vector_array_t& stateTrajectory,
                                   const input_vector_array_t& inputTrajectory, scalar_t& totalCost);

  /**
   * Calculates an LQ approximate of the optimal control problem for the nodes.
   *
   * @param [in] timeTrajectory: The time trajectory.
   * @param [in] postEventIndices: The post event indices.
   * @param [in] stateTrajectory: The state trajectory.
   * @param [in] inputTrajectory: The input trajectory.
   * @param modelDataTrajectory: The model data trajectory.
   */
  virtual void approximateIntermediateLQ(const scalar_array_t& timeTrajectory, const size_array_t& postEventIndices,
                                         const state_vector_array_t& stateTrajectory, const input_vector_array_t& inputTrajectory,
                                         ModelDataBase::array_t& modelDataTrajectory) = 0;

  /**
   * Augments the cost function for the given model data.
   *
   * @param [in] workerIndex: Working agent index.
   * @param modelData: The model data.
   */
  void augmentCostWorker(size_t workerIndex, ModelDataBase& modelData);

  /**
   * Takes the following steps: (1) Computes the Hessian of the Hamiltonian (i.e., Hm) (2) Based on Hm, it calculates
   * the range space and the null space projections of the input-state equality constraints. (3) Based on these two
   * projections, defines the projected LQ model. (4) Finally, defines the Riccati equation modifiers based on the
   * search strategy.
   *
   * @param [in] strategy: The search strategy e.g., LINE_SEARCH.
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @param [out] projectedModelData: The projected model data.
   * @param [out] riccatiModification: The Riccati equation modifier.
   */
  void computeProjectionAndRiccatiModification(DDP_Strategy strategy, const ModelDataBase& modelData, const dynamic_matrix_t& Sm,
                                               ModelDataBase& projectedModelData, RiccatiModificationBase& riccatiModification) const;

  /**
   * Computes Hessian of the Hamiltonian.
   *
   * @param [in] strategy: The search strategy e.g., LINE_SEARCH.
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @return The Hessian matrix of the Hamiltonian.
   */
  virtual dynamic_matrix_t computeHamiltonianHessian(DDP_Strategy strategy, const ModelDataBase& modelData,
                                                     const state_matrix_t& Sm) const = 0;

  /**
   *
   * @param [in] Hm: inv(Hm) defines the oblique projection for state-input equality constraints.
   * @param [in] Dm: The derivative of the state-input constraints w.r.t. input.
   * @param [out] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [out] constraintNullProjector: The projection matrix to the null space of constrained.
   */
  void computeProjections(const dynamic_matrix_t& Hm, const dynamic_matrix_t& Dm, dynamic_matrix_t& constraintRangeProjector,
                          dynamic_matrix_t& constraintNullProjector) const;

  /**
   * Computes the Riccati modification based on the strategy.
   *
   * @param [in] strategy: The search strategy e.g., LINE_SEARCH
   * @param [in] projectedModelData: The projected data model
   * @param [out] deltaQm: The Riccati modifier to cost 2nd derivative w.r.t. state.
   * @param [out] deltaGv: The Riccati modifier to cost derivative w.r.t. input.
   * @param [out] deltaGm: The Riccati modifier to cost input-state derivative.
   */
  void computeRiccatiModification(DDP_Strategy strategy, const ModelDataBase& projectedModelData, dynamic_matrix_t& deltaQm,
                                  dynamic_vector_t& deltaGv, dynamic_matrix_t& deltaGm) const;

  /**
   * Projects the unconstrained LQ coefficients to constrained ones.
   *
   * @param [in] modelData: The model data.
   * @param [in] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [in] constraintNullProjector: The projection matrix to the null space of constrained.
   * @param [out] projectedModelData: The projected model data.
   */
  void projectLQ(const ModelDataBase& modelData, const dynamic_matrix_t& constraintRangeProjector,
                 const dynamic_matrix_t& constraintNullProjector, ModelDataBase& projectedModelData) const;

  /**
   * Shifts the Hessian based on the strategy defined by Line_Search::hessianCorrectionStrategy_.
   *
   * @tparam Derived type.
   * @param matrix: The Hessian matrix.
   */
  template <typename Derived>
  void shiftHessian(Eigen::MatrixBase<Derived>& matrix) const;

  /**
   * Calculates controller at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index
   * @param [in] timeIndex: Time index in the partition
   */
  virtual void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) = 0;

  /**
   * Performs one rollout while only the input correction for the type-1 constraint is considered.
   */
  virtual void baselineRollout();

  /**
   * Defines line search task on a thread with various learning rates and choose the largest acceptable step-size.
   */
  void lineSearchTask();

  /**
   * Line search with a specific learning rate.
   *
   * @param [in] workerIndex
   * @param [in] stepLength
   * @param [out] totalCost
   * @param [out] stateInputEqConstraintISE
   * @param [out] stateEqConstraintISE
   * @param [out] stateEqFinalConstraintISE
   * @param [out] inequalityConstraintPenalty
   * @param [out] inequalityConstraintISE
   * @param [out] controllersStock
   * @param [out] timeTrajectoriesStock
   * @param [out] postEventIndicesStock
   * @param [out] stateTrajectoriesStock
   * @param [out] inputTrajectoriesStock
   * @param [out] modelDataTrajectoriesStock
   */
  void lineSearchWorker(size_t workerIndex, scalar_t stepLength, scalar_t& totalCost, scalar_t& stateInputEqConstraintISE,
                        scalar_t& stateEqConstraintISE, scalar_t& stateEqFinalConstraintISE, scalar_t& inequalityConstraintPenalty,
                        scalar_t& inequalityConstraintISE, linear_controller_array_t& controllersStock,
                        scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
                        state_vector_array2_t& stateTrajectoriesStock, input_vector_array2_t& inputTrajectoriesStock,
                        ModelDataBase::array2_t& modelDataTrajectoriesStock);

  /**
   * Calculates state-input constraints ISE (Integral of Square Error).
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp.
   * @param [in] nc1TrajectoriesStock: Array of trajectories containing the number of the active constraints.
   * @param [in] EvTrajectoriesStock: Array of trajectories containing the value of the constraints.
   * @return The constraints ISE.
   */
  scalar_t calculateConstraintISE(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& nc1TrajectoriesStock,
                                  const constraint1_vector_array2_t& EvTrajectoriesStock) const;

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
   * Calculates max feedforward update norm and max type-1 error update norm.
   *
   * @param maxDeltaUffNorm: max feedforward update norm.
   * @param maxDeltaUeeNorm: max type-1 error update norm.
   */
  void calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm);

  /**
   * Caches the nominal trajectories.
   */
  void swapNominalTrajectoriesToCache();

  /**
   * Display rollout info and scores.
   */
  void printRolloutInfo();

 private:
  /**
   * Corrects the initial caching of the nominal trajectories.
   * This is necessary for:
   *   + The moving horizon (MPC) application
   *   + The very first call of the algorithm where there is no previous nominal trajectories.
   */
  void correctInitcachedNominalTrajectories();

  /**
   * Corrects for the tail of the cached trajectory based on the nominal trajectory. This compensates for the
   * the moving horizon (MPC) applications where the final time of the cached trajectory is smaller than the
   * nominal one.
   *
   * @param [in] timeSegment: The interval index and interpolation coefficient alpha of the cached trajectory final
   * time in the nominal time trajectory.
   * @param [in] currentTrajectory: The nominal trajectory.
   * @param [out] cachedTrajectory: The cached trajectory.
   */
  template <typename Data_T, class Alloc>
  static void correctcachedTrajectoryTail(std::pair<int, scalar_t> timeSegment, const std::vector<Data_T, Alloc>& currentTrajectory,
                                          std::vector<Data_T, Alloc>& cachedTrajectory);

  void runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const controller_ptr_array_t& controllersPtrStock) override;

 protected:
  // Variables
  DDP_Settings ddpSettings_;

  ThreadPool threadPool_;

  // multi-threading helper variables
  std::atomic_size_t nextTaskId_;
  std::atomic_size_t nextTimeIndex_;

  std::string algorithmName_;

  unsigned long long int rewindCounter_;

  bool useParallelRiccatiSolverFromInitItr_ = false;

  scalar_t initTime_;
  scalar_t finalTime_;
  state_vector_t initState_;

  size_t initActivePartition_;
  size_t finalActivePartition_;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  // trajectory spreading
  TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM> trajectorySpreadingController_;

  std::atomic_size_t iteration_;
  scalar_array_t iterationCost_;
  scalar_array_t iterationISE1_;
  scalar_array_t iterationISE2_;

  scalar_t nominalTotalCost_;
  scalar_t stateInputEqConstraintISE_;
  scalar_t stateEqConstraintISE_;
  scalar_t stateEqFinalConstraintISE_;
  scalar_t inequalityConstraintPenalty_;
  scalar_t inequalityConstraintISE_;

  // forward pass and backward pass average time step
  scalar_t avgTimeStepFP_;
  scalar_t avgTimeStepBP_;

  std::vector<std::unique_ptr<rollout_base_t>> dynamicsForwardRolloutPtrStock_;
  std::vector<std::unique_ptr<rollout_base_t>> operatingTrajectoriesRolloutPtrStock_;
  std::vector<std::unique_ptr<linear_quadratic_approximator_t>> linearQuadraticApproximatorPtrStock_;
  std::vector<typename cost_function_base_t::Ptr> heuristicsFunctionsPtrStock_;
  std::vector<std::shared_ptr<penalty_base_t>> penaltyPtrStock_;

  scalar_t nominalControllerUpdateIS_ = 0.0;
  linear_controller_array_t nominalControllersStock_;
  bool isInitInternalControllerEmpty_;

  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;

  // used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating run()
  scalar_t cachedControllerUpdateIS_ = 0.0;
  linear_controller_array_t cachedControllersStock_;
  scalar_array2_t cachedTimeTrajectoriesStock_;
  size_array2_t cachedPostEventIndicesStock_;
  state_vector_array2_t cachedStateTrajectoriesStock_;
  input_vector_array2_t cachedInputTrajectoriesStock_;

  // intermediate model data trajectory
  ModelDataBase::array2_t modelDataTrajectoriesStock_;
  ModelDataBase::array2_t cachedModelDataTrajectoriesStock_;

  // event times model data
  ModelDataBase::array2_t modelDataEventTimesStock_;
  ModelDataBase::array2_t cachedModelDataEventTimesStock_;

  // projected model data trajectory
  ModelDataBase::array2_t projectedModelDataTrajectoriesStock_;
  ModelDataBase::array2_t cachedProjectedModelDataTrajectoriesStock_;

  // Riccati modification
  RiccatiModificationBase::array2_t riccatiModificationTrajectoriesStock_;
  RiccatiModificationBase::array2_t cachedRiccatiModificationTrajectoriesStock_;

  // Riccati solution coefficients
  scalar_array2_t SsTimeTrajectoryStock_;
  scalar_array2_t SsNormalizedTimeTrajectoryStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  scalar_array2_t sTrajectoryStock_;
  state_vector_array2_t SvTrajectoryStock_;
  state_matrix_array2_t SmTrajectoryStock_;

  scalar_array_t sFinalStock_;
  state_vector_array_t SvFinalStock_;
  state_matrix_array_t SmFinalStock_;
  state_vector_array_t xFinalStock_;

  scalar_t sHeuristics_;
  state_vector_t SvHeuristics_;
  state_matrix_t SmHeuristics_;

  // Line-Search
  struct LineSearchImpl {
    scalar_t baselineTotalCost;                      // the cost of the rollout for zero learning rate
    std::atomic<scalar_t> stepLengthStar;            // the optimal step length.
    linear_controller_array_t initControllersStock;  // needed for lineSearch

    std::atomic_size_t alphaExpNext;
    std::vector<bool> alphaProcessed;
    std::mutex lineSearchResultMutex;

  } lineSearchImpl_;

  // Levenberg-Marquardt
  struct LevenbergMarquardtImpl {
    scalar_t pho = 1.0;                           // the ratio between actual reduction and predicted reduction
    scalar_t riccatiMultiple = 0.0;               // the Riccati multiple for Tikhonov regularization.
    scalar_t riccatiMultipleAdaptiveRatio = 1.0;  // the adaptive ratio of geometric progression for Riccati multiple.
    size_t numSuccessiveRejections = 0;           // the number of successive rejections of solution.

  } levenbergMarquardtImpl_;

  std::vector<int> startingIndicesRiccatiWorker_;
  std::vector<int> endingIndicesRiccatiWorker_;
  // parallel Riccati solver
  std::mutex riccatiSolverDataMutex_;

  // benchmarking
  benchmark::RepeatedTimer forwardPassTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer backwardPassTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
  benchmark::RepeatedTimer searchStrategyTimer_;
};

}  // namespace ocs2

#include "implementation/DDP_BASE.h"
