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
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/misc/ThreadPool.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_solver/Solver_BASE.h>
#include <ocs2_oc/rollout/OperatingTrajectoriesRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/Rollout_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_ddp/DDP_Settings.h>

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
  using typename BASE::dynamic_matrix_t;
  using typename BASE::dynamic_vector_array_t;
  using typename BASE::dynamic_vector_t;
  using typename BASE::eigen_scalar_array2_t;
  using typename BASE::eigen_scalar_array_t;
  using typename BASE::eigen_scalar_t;
  using typename BASE::input_constraint1_matrix_array2_t;
  using typename BASE::input_constraint1_matrix_array_t;
  using typename BASE::input_constraint1_matrix_t;
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
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite
   * time optimal control formulation. If it is not defined, we will use the
   * terminal cost function defined in costFunctionPtr.
   */
  DDP_BASE(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr, const constraint_base_t* systemConstraintsPtr,
           const cost_function_base_t* costFunctionPtr, const operating_trajectories_base_t* operatingTrajectoriesPtr,
           const DDP_Settings& ddpSettings, const cost_function_base_t* heuristicsFunctionPtr, const char* algorithmName);

  /**
   * Destructor.
   */
  virtual ~DDP_BASE();

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
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the
   * output time trajectory stamp.
   * @param [out] postEventIndicesStock: Array of the post-event indices.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the
   * output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the
   * output control input trajectory.
   * @param [in] threadId: Working thread (default is 0).
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(linear_controller_array_t& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                             size_array2_t& postEventIndicesStock, state_vector_array2_t& stateTrajectoriesStock,
                             input_vector_array2_t& inputTrajectoriesStock, size_t threadId = 0);

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
   * Calculates cost of a rollout.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the time trajectory stamp of a rollout.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the state trajectory of a rollout.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the control input trajectory of a rollout.
   * @param [in] threadId: Working thread (default is 0).
   * @return totalCost: The total cost of the rollout.
   */
  scalar_t calculateRolloutCost(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                const state_vector_array2_t& stateTrajectoriesStock, const input_vector_array2_t& inputTrajectoriesStock,
                                size_t threadId = 0);

  /**
   * Calculates the merit function as a function of cost, inequality constraints penalty, and ISE of state equality constraints.
   *
   * @param [in] nc2FinalStock: Array containing the number of the active final state-only constraints.
   * @param [in] HvFinalStock: Array containing the value of the final state-only constraints.
   * @param [in] performanceIndex: The performance index of the rollout.
   * @param [in] threadId: Working thread (default is 0).
   */
  void calculateRolloutMerit(const size_array2_t& nc2FinalStock, const constraint2_vector_array2_t& HvFinalStock,
                             PerformanceIndex& performanceIndex, size_t threadId = 0) const;

  /**
   * Approximates the nonlinear problem as a linear-quadratic problem around the
   * nominal state and control trajectories. This method updates the following
   * variables:
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
   * 	- intermediate cost: \f$ q(t) + 0.5 xQ_m(t)x + x'Q_v(t) + u'P_m(t)x +
   * 0.5u'R_m(t)u + u'R_v(t) \f$
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
   * - nominalControllersStock_: the controller that stabilizes the system
   * around the new nominal trajectory and improves the constraints as well as
   * the increment to the feed-forward control input.
   */
  virtual void calculateController();

  /**
   * Line search on the feedforward parts of the controller. It uses the
   * following approach for line search: The constraint TYPE-1 correction term
   * is directly added through a user defined stepSize (defined in
   * settings_.constraintStepSize_). But the cost minimization term is optimized
   * through a line-search strategy defined in ILQR settings.
   */
  virtual void lineSearch();

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
                                                   const eigen_scalar_t& sFinal);

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

  const PerformanceIndex& getPerformanceIndeces() const override;

  size_t getNumIterations() const override;

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

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
   * Calculates an LQ approximate of the optimal control problem at a given
   * partition and a node.
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
   * @return totalCost: The total cost.
   */
  scalar_t calculateCostWorker(size_t workerIndex, size_t partitionIndex, const scalar_array_t& timeTrajectory,
                               const size_array_t& eventsPastTheEndIndeces, const state_vector_array_t& stateTrajectory,
                               const input_vector_array_t& inputTrajectory);

  /**
   * Calculates an LQ approximate of the unconstrained optimal control problem
   * at a given partition and a node.
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
   * @param workerIndex
   * @param learningRate
   * @param lsPerformanceIndex
   * @param lsConstraint1MaxNorm
   * @param lsConstraint2MaxNorm
   * @param lsControllersStock
   * @param lsTimeTrajectoriesStock
   * @param lsPostEventIndicesStock
   * @param lsStateTrajectoriesStock
   * @param lsInputTrajectoriesStock
   */
  void lineSearchWorker(size_t workerIndex, scalar_t learningRate, PerformanceIndex& lsPerformanceIndex, scalar_t& lsConstraint1MaxNorm,
                        scalar_t& lsConstraint2MaxNorm, linear_controller_array_t& lsControllersStock,
                        scalar_array2_t& lsTimeTrajectoriesStock, size_array2_t& lsPostEventIndicesStock,
                        state_vector_array2_t& lsStateTrajectoriesStock, input_vector_array2_t& lsInputTrajectoriesStock);

  /**
   * Solves Riccati equations for the partitions assigned to the given thread.
   */
  virtual void riccatiSolverTask() = 0;

  /**
   * compute the merit function for given rollout
   *
   * @param [in] timeTrajectoriesStock: simulation time trajectory
   * @param [in] nc1TrajectoriesStock: rollout's number of active constraints in
   * each time step
   * @param [in] EvTrajectoryStock: rollout's constraints value
   * @param [in] lagrangeTrajectoriesStock: constraint Lagrange multiplier for
   * the given rollout
   * @param [in] totalCost: the total cost of the trajectory
   * @param [out] meritFunctionValue: the total merit function value of the
   * trajectory
   * @param [out] constraintISE: Integral of Square Error (ISE)
   */
  void calculateMeritFunction(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& nc1TrajectoriesStock,
                              const constraint1_vector_array2_t& EvTrajectoryStock,
                              const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock, scalar_t totalCost,
                              scalar_t& meritFunctionValue, scalar_t& constraintISE);

  /**
   * Calculates state-input constraints ISE (Integral of Square Error). It also
   * return the maximum norm of the constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the
   * time trajectory stamp.
   * @param [in] nc1TrajectoriesStock: Array of trajectories containing the
   * number of the active state-input constraints.
   * @param [in] EvTrajectoriesStock: Array of trajectories containing the value
   * of the state-input constraints.
   * @param [out] constraintISE: The state-input constraints ISE.
   * @return maximum norm of the constraints.
   */
  scalar_t calculateConstraintISE(const scalar_array2_t& timeTrajectoriesStock,
                                  const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
                                  const constraint1_vector_array2_t& EvTrajectoriesStock, scalar_t& constraintISE);

  /**
   * Calculate integrated penalty from inequality constraints.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the
   * time trajectory stamp.
   * @param [in] ncIneqTrajectoriesStock: Array of trajectories containing the
   * number of inequalityConstraints
   * @param [in] hTrajectoriesStock: Array of trajectories containing the value
   * of the inequality constraints.
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

  std::atomic<scalar_t> learningRateStar_;  // The optimal learning rate.
  scalar_t maxLearningRate_ = 1.0;          // The maximum permitted learning rate
                                            // (settings_.maxLearningRateSLQ_).

  std::vector<int> startingIndicesRiccatiWorker_;
  std::vector<int> endingIndicesRiccatiWorker_;

  // trajectory spreading
  TrajectorySpreadingControllerAdjustment<STATE_DIM, INPUT_DIM> trajectorySpreadingController_;

  std::atomic_size_t iteration_;

  scalar_t nominalConstraint1MaxNorm_;
  scalar_t nominalConstraint2MaxNorm_;

  PerformanceIndex performanceIndex_;
  std::vector<PerformanceIndex> performanceIndexHistory_;

  // Forward pass and backward pass average time step
  scalar_t avgTimeStepFP_;
  scalar_t avgTimeStepBP_;

  std::vector<std::unique_ptr<rollout_base_t>> dynamicsForwardRolloutPtrStock_;
  std::vector<std::unique_ptr<rollout_base_t>> operatingTrajectoriesRolloutPtrStock_;
  std::vector<std::unique_ptr<linear_quadratic_approximator_t>> linearQuadraticApproximatorPtrStock_;
  std::vector<typename cost_function_base_t::Ptr> heuristicsFunctionsPtrStock_;
  std::vector<std::shared_ptr<penalty_base_t>> penaltyPtrStock_;

  linear_controller_array_t nominalControllersStock_;

  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  state_vector_array2_t nominalStateTrajectoriesStock_;
  input_vector_array2_t nominalInputTrajectoriesStock_;

  // Used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating run()
  scalar_array2_t cachedTimeTrajectoriesStock_;
  size_array2_t cachedPostEventIndicesStock_;
  state_vector_array2_t cachedStateTrajectoriesStock_;
  input_vector_array2_t cachedInputTrajectoriesStock_;

  // line search
  std::mutex lineSearchResultMutex_;
  std::atomic_size_t alphaExpNext_;
  std::vector<bool> alphaProcessed_;
  scalar_t baselineMerit_;                            // the merit of the rollout for zero learning rate
  linear_controller_array_t initLScontrollersStock_;  // needed for lineSearch

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

  // benchmarking
  benchmark::RepeatedTimer forwardPassTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer backwardPassTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
  benchmark::RepeatedTimer linesearchTimer_;
};

}  // namespace ocs2

#include "implementation/DDP_BASE.h"
