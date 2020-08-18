/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/control/TrajectorySpreadingControllerAdjustment.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/integration/TrapezoidalIntegration.h>
#include <ocs2_core/misc/Benchmark.h>
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

#include "DDP_Settings.h"
#include "riccati_equations/RiccatiModification.h"

namespace ocs2 {

/**
 * This class is an interface class for the Gauss-Newton DDP based methods.
 */
class GaussNewtonDDP : public Solver_BASE {
 public:
  // Line-Search
  struct LineSearchModule {
    scalar_t baselineMerit = 0.0;                        // the merit of the rollout for zero learning rate
    std::atomic<scalar_t> stepLengthStar{0.0};           // the optimal step length.
    std::vector<LinearController> initControllersStock;  // needed for lineSearch

    std::atomic_size_t alphaExpNext{0};
    std::vector<bool> alphaProcessed;
    std::mutex lineSearchResultMutex;
  };

  // Levenberg-Marquardt
  struct LevenbergMarquardtModule {
    scalar_t pho = 1.0;                           // the ratio between actual reduction and predicted reduction
    scalar_t riccatiMultiple = 0.0;               // the Riccati multiple for Tikhonov regularization.
    scalar_t riccatiMultipleAdaptiveRatio = 1.0;  // the adaptive ratio of geometric progression for Riccati multiple.
    size_t numSuccessiveRejections = 0;           // the number of successive rejections of solution.
  };

  struct ConstraintPenaltyCoefficients {
    scalar_t stateEqConstrPenaltyTol = 1e-3;
    scalar_t stateEqConstrPenaltyCoeff = 0.0;

    scalar_t stateFinalEqConstrPenaltyTol = 1e-3;
    scalar_t stateFinalEqConstrPenaltyCoeff = 0.0;

    scalar_t stateInputEqConstrPenaltyTol = 1e-3;
    scalar_t stateInputEqConstrPenaltyCoeff = 0.0;
  };

  /**
   * class for collecting SLQ data
   */
  friend class DDP_DataCollector;

  /**
   * Constructor
   *
   * @param [in] rolloutPtr: The rollout class used for simulating the system dynamics.
   * @param [in] systemDynamicsPtr: The system dynamics and derivatives for the subsystems.
   * @param [in] systemConstraintsPtr: The system constraint function and its derivatives for subsystems.
   * @param [in] costFunctionPtr: The cost function (intermediate and final costs) and its derivatives for subsystems.
   * @param [in] operatingTrajectoriesPtr: The operating trajectories of system which will be used for initialization.
   * @param [in] ddpSettings: Structure containing the settings for the Gauss-Newton DDP algorithm.
   * @param [in] heuristicsFunctionPtr: Heuristic function used in the infinite time optimal control formulation.
   * If it is not defined, we will use the final cost function defined in costFunctionPtr.
   */
  GaussNewtonDDP(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                 const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
                 ddp::Settings ddpSettings, const CostFunctionBase* heuristicsFunctionPtr);

  /**
   * Destructor.
   */
  ~GaussNewtonDDP() override;

  void reset() override;

  size_t getNumIterations() const override;

  scalar_t getFinalTime() const override;

  const scalar_array_t& getPartitioningTimes() const override;

  const PerformanceIndex& getPerformanceIndeces() const override;

  const std::vector<PerformanceIndex>& getIterationsLog() const override;

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const final;

  scalar_t getValueFunction(scalar_t time, const vector_t& state) const override;

  void getValueFunctionStateDerivative(scalar_t time, const vector_t& state, vector_t& Vx) const override;

  void getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state, vector_t& nu) const override;

  void rewindOptimizer(size_t firstIndex) override;

  void printBenchmarkingInfo() const override;

  const unsigned long long int& getRewindCounter() const override { return rewindCounter_; }

  /**
   * Write access to ddp settings
   */
  ddp::Settings& settings() { return ddpSettings_; }

  /**
   * Const access to ddp settings
   */
  const ddp::Settings& settings() const { return ddpSettings_; }

  /**
   * Upon activation in the multi-thread Gauss-Newton DDP class (DDP_MT), the parallelization
   * of the backward pass takes place from the the first iteration which
   * normally become effective after the first iteration.
   *
   * @param [in] flag: If set true, the parallel Riccati solver will be used
   * from the first iteration.
   */
  void useParallelRiccatiSolverFromInitItr(bool flag) { useParallelRiccatiSolverFromInitItr_ = flag; }

  /**
   * Computes the normalized time for Riccati backward pass.
   *
   * @param [in] timeTrajectory: The time trajectory.
   * @param [in] postEventIndices: The post event indices.
   * @param [out] normalizedTimeTrajectory: The reversed and negated timeTrajectory.
   * @param [out] normalizedPostEventIndices: The corresponding post event indices of normalizedTimeTrajectory.
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

 protected:
  /**
   * Sets up optimizer for different number of partitions.
   *
   * @param [in] numPartitions: number of partitions.
   */
  virtual void setupOptimizer(size_t numPartitions);

  /**
   * Helper to run task multiple times in parallel (blocking)
   *
   * @param [in] taskFunction: task function
   * @param [in] N: number of times to run taskFunction, if N = 1 it is run in the main thread
   */
  void runParallel(std::function<void(void)> taskFunction, size_t N);

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
  void computeProjectionAndRiccatiModification(ddp_strategy::type strategy, const ModelDataBase& modelData, const matrix_t& Sm,
                                               ModelDataBase& projectedModelData, riccati_modification::Data& riccatiModification) const;

  /**
   * Computes Hessian of the Hamiltonian.
   *
   * @param [in] strategy: The search strategy e.g., LINE_SEARCH.
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @return The Hessian matrix of the Hamiltonian.
   */
  virtual matrix_t computeHamiltonianHessian(ddp_strategy::type strategy, const ModelDataBase& modelData, const matrix_t& Sm) const = 0;

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
                                         const vector_array_t& stateTrajectory, const vector_array_t& inputTrajectory,
                                         std::vector<ModelDataBase>& modelDataTrajectory) = 0;

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
   * Calculates controller at a given partition and a node.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: Time partition index
   * @param [in] timeIndex: Time index in the partition
   */
  virtual void calculateControllerWorker(size_t workerIndex, size_t partitionIndex, size_t timeIndex) = 0;

  /**
   * Solves Riccati equations for all the partitions.
   *
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   *
   * @return average time step
   */
  virtual scalar_t solveSequentialRiccatiEquations(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal) = 0;

  /**
   * The implementation for solving Riccati equations for all the partitions.
   *
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   *
   * @return average time step
   */
  scalar_t solveSequentialRiccatiEquationsImpl(const matrix_t& SmFinal, const vector_t& SvFinal, const scalar_t& sFinal);

  /**
   * Solves a set of Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] partitionIndex: The requested partition index to solve Riccati equations.
   * @param [in] SmFinal: The final Sm for Riccati equation.
   * @param [in] SvFinal: The final Sv for Riccati equation.
   * @param [in] sFinal: The final s for Riccati equation.
   */
  virtual void riccatiEquationsWorker(size_t workerIndex, size_t partitionIndex, const matrix_t& SmFinal, const vector_t& SvFinal,
                                      const scalar_t& sFinal) = 0;

 private:
  /**
   * Distributes the sequential tasks (e.g. Riccati solver) in between threads.
   */
  void distributeWork();

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
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   * @param [in] workerIndex: Working thread (default is 0).
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(std::vector<LinearController>& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                             size_array2_t& postEventIndicesStock, vector_array2_t& stateTrajectoriesStock,
                             vector_array2_t& inputTrajectoriesStock, std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                             std::vector<std::vector<ModelDataBase>>& modelDataEventTimesStock, size_t workerIndex = 0);

  /**
   * Display rollout info and scores.
   */
  void printRolloutInfo() const;

  /**
   * Evaluates cost and constraints along the given time trajectories.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   * @param [out] heuristicsValue: The Heuristics function value.
   * @param [in] workerIndex: Working thread (default is 0).
   */
  void rolloutCostAndConstraints(const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                 const vector_array2_t& stateTrajectoriesStock, const vector_array2_t& inputTrajectoriesStock,
                                 std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                                 std::vector<std::vector<ModelDataBase>>& modelDataEventTimesStock, scalar_t& heuristicsValue,
                                 size_t workerIndex = 0);

  /**
   * Calculates constraints ISE (Integral of Square Error), cost function integral, and the merit function.
   *
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [in] modelDataEventTimesStock: Array of model data at event times.
   * @param [in] heuristicsValue: The Heuristics function value.
   * @param [in] workerIndex: Working thread (default is 0).
   *
   * @return The cost, merit function and ISEs of constraints for the trajectory.
   */
  PerformanceIndex calculateRolloutPerformanceIndex(const scalar_array2_t& timeTrajectoriesStock,
                                                    const std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                                                    const std::vector<std::vector<ModelDataBase>>& modelDataEventTimesStock,
                                                    scalar_t heuristicsValue, size_t workerIndex = 0);

  /**
   * Performs a full rollout of dynamics, cost, and constraints with a given step length.
   *
   * @param [in] workerIndex: The index of the worker.
   * @param [in] stepLength: The step length for which the controller is updated based on the Gauss-Newton DDP solution.
   * @param [out] controllersStock: The updated array of control policies based on the step length.
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] postEventIndicesStock: Array of the post-event indices.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   * @param [out] performanceIndex: The cost, merit function and ISEs of constraints for the trajectory.
   *
   * @return average time step.
   */
  scalar_t performFullRollout(size_t workerIndex, scalar_t stepLength, std::vector<LinearController>& controllersStock,
                              scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
                              vector_array2_t& stateTrajectoriesStock, vector_array2_t& inputTrajectoriesStock,
                              std::vector<std::vector<ModelDataBase>>& modelDataTrajectoriesStock,
                              std::vector<std::vector<ModelDataBase>>& modelDataEventTimesStock, PerformanceIndex& performanceIndex);

  /**
   * Line search on the feedforward parts of the controller. It chooses the largest acceptable step-size.
   * The class computes the nominal controller and the nominal trajectories as well the corresponding performance indices.
   */
  void lineSearch(LineSearchModule& lineSearchModule);

  /**
   * Defines line search task on a thread with various learning rates and choose the largest acceptable step-size.
   * The class computes the nominal controller and the nominal trajectories as well the corresponding performance indices.
   */
  void lineSearchTask(LineSearchModule& lineSearchModule);

  /**
   * Calculates the merit function based on the performance index .
   *
   * @param The performance index which includes the merit, cost, and ISEs of constraints.
   */
  void calculateRolloutMerit(PerformanceIndex& performanceIndex) const;

  /**
   * Levenberg Marquardt strategy.
   * The class computes the nominal controller and the nominal trajectories as well the corresponding performance indices.
   */
  void levenbergMarquardt(LevenbergMarquardtModule& levenbergMarquardtModule);

  /**
   * Solves Riccati equations for the partitions assigned to the given thread.
   */
  void riccatiSolverTask();

  /**
   * Calculates the integral of the squared (IS) norm of the controller update.
   *
   * @param [in] controllersStock: An array of controllers.
   * @retuen The integral of the squared (IS) norm of the controller update.
   */
  scalar_t calculateControllerUpdateIS(const std::vector<LinearController>& controllersStock) const;

  /**
   * Calculates max feedforward update norm and max type-1 error update norm.
   *
   * @param maxDeltaUffNorm: max feedforward update norm.
   * @param maxDeltaUeeNorm: max type-1 error update norm.
   */
  void calculateControllerUpdateMaxNorm(scalar_t& maxDeltaUffNorm, scalar_t& maxDeltaUeeNorm) const;

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
   *
   * @param [in] Hm: inv(Hm) defines the oblique projection for state-input equality constraints.
   * @param [in] Dm: The derivative of the state-input constraints w.r.t. input.
   * @param [out] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [out] constraintNullProjector: The projection matrix to the null space of constrained.
   */
  void computeProjections(const matrix_t& Hm, const matrix_t& Dm, matrix_t& constraintRangeProjector,
                          matrix_t& constraintNullProjector) const;

  /**
   * Computes the Riccati modification based on the strategy.
   *
   * @param [in] strategy: The search strategy e.g., LINE_SEARCH
   * @param [in] projectedModelData: The projected data model
   * @param [out] deltaQm: The Riccati modifier to cost 2nd derivative w.r.t. state.
   * @param [out] deltaGv: The Riccati modifier to cost derivative w.r.t. input.
   * @param [out] deltaGm: The Riccati modifier to cost input-state derivative.
   */
  void computeRiccatiModification(ddp_strategy::type strategy, const ModelDataBase& projectedModelData, matrix_t& deltaQm,
                                  vector_t& deltaGv, matrix_t& deltaGm) const;

  /**
   * Projects the unconstrained LQ coefficients to constrained ones.
   *
   * @param [in] modelData: The model data.
   * @param [in] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [in] constraintNullProjector: The projection matrix to the null space of constrained.
   * @param [out] projectedModelData: The projected model data.
   */
  void projectLQ(const ModelDataBase& modelData, const matrix_t& constraintRangeProjector, const matrix_t& constraintNullProjector,
                 ModelDataBase& projectedModelData) const;

  /**
   * Shifts the Hessian based on the strategy defined by Line_Search::hessianCorrectionStrategy_.
   *
   * @param matrix: The Hessian matrix.
   */
  void shiftHessian(matrix_t& matrix) const;

  /**
   * Augments the cost function for the given model data.
   *
   * @param [in] workerIndex: Working agent index.
   * @param [in] stateEqConstrPenaltyCoeff: The state-only equality penalty coefficient of the Augmented Lagrangian method.
   * @param [in] stateInputEqConstrPenaltyCoeff: The state-input equality penalty coefficient of the Augmented Lagrangian method.
   * @param modelData: The model data.
   */
  void augmentCostWorker(size_t workerIndex, scalar_t stateEqConstrPenaltyCoeff, scalar_t stateInputEqConstrPenaltyCoeff,
                         ModelDataBase& modelData) const;

  /**
   * Initialize the constraint penalty coefficients.
   */
  void initializeConstraintPenalties();

  /**
   * Updates the constraint penalty coefficients.
   * @param [in] stateEqConstraintISE: ISE of the intermediate state-only equality constraints.
   * @param [in] stateEqFinalConstraintSSE: SSE of the state-only equality constraints at event times.
   * @param [in] stateInputEqConstraintISE: ISE of the intermediate state-input equality constraints.
   */
  void updateConstraintPenalties(scalar_t stateEqConstraintISE, scalar_t stateEqFinalConstraintSSE, scalar_t stateInputEqConstraintISE);

  /**
   * Caches the iteration's data.
   */
  void swapDataToCache();

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

  /**
   * Runs the initialization method for Gauss-Newton DDP.
   */
  void runInit();

  /**
   * Runs a single iteration of Gauss-Newton DDP.
   */
  void runIteration();

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
               const std::vector<ControllerBase*>& controllersPtrStock) override;

 protected:
  // multi-threading helper variables
  std::atomic_size_t nextTaskId_{0};
  std::atomic_size_t nextTimeIndex_{0};

  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  size_t initActivePartition_ = 0;
  size_t finalActivePartition_ = 0;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;

  std::vector<std::unique_ptr<LinearQuadraticApproximator>> linearQuadraticApproximatorPtrStock_;

  // optimized controller
  std::vector<LinearController> nominalControllersStock_;

  // optimized trajectories
  scalar_array2_t nominalTimeTrajectoriesStock_;
  size_array2_t nominalPostEventIndicesStock_;
  vector_array2_t nominalStateTrajectoriesStock_;
  vector_array2_t nominalInputTrajectoriesStock_;

  // intermediate model data trajectory
  std::vector<std::vector<ModelDataBase>> modelDataTrajectoriesStock_;

  // event times model data
  std::vector<std::vector<ModelDataBase>> modelDataEventTimesStock_;

  // projected model data trajectory
  std::vector<std::vector<ModelDataBase>> projectedModelDataTrajectoriesStock_;

  // Riccati modification
  std::vector<std::vector<riccati_modification::Data>> riccatiModificationTrajectoriesStock_;

  // Riccati solution coefficients
  scalar_array2_t SsTimeTrajectoryStock_;
  scalar_array2_t SsNormalizedTimeTrajectoryStock_;
  size_array2_t SsNormalizedEventsPastTheEndIndecesStock_;
  scalar_array2_t sTrajectoryStock_;
  vector_array2_t SvTrajectoryStock_;
  matrix_array2_t SmTrajectoryStock_;

  // Line-Search
  LineSearchModule lineSearchModule_;

  // Levenberg-Marquardt
  LevenbergMarquardtModule levenbergMarquardtModule_;

 private:
  ddp::Settings ddpSettings_;

  std::unique_ptr<ThreadPool> threadPoolPtr_;

  unsigned long long int rewindCounter_ = 0;

  bool useParallelRiccatiSolverFromInitItr_ = false;

  // trajectory spreading
  TrajectorySpreadingControllerAdjustment trajectorySpreadingController_;

  std::atomic_size_t iteration_{0};

  PerformanceIndex performanceIndex_;

  std::vector<PerformanceIndex> performanceIndexHistory_;

  // forward pass and backward pass average time step
  scalar_t avgTimeStepFP_ = 0.0;
  scalar_t avgTimeStepBP_ = 0.0;

  std::vector<std::unique_ptr<RolloutBase>> dynamicsForwardRolloutPtrStock_;
  std::vector<std::unique_ptr<RolloutBase>> operatingTrajectoriesRolloutPtrStock_;
  std::vector<std::unique_ptr<CostFunctionBase>> heuristicsFunctionsPtrStock_;
  std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrStock_;

  scalar_t nominalControllerUpdateIS_ = 0.0;

  // used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating run()
  scalar_t cachedControllerUpdateIS_ = 0.0;
  std::vector<LinearController> cachedControllersStock_;
  scalar_array2_t cachedTimeTrajectoriesStock_;
  size_array2_t cachedPostEventIndicesStock_;
  vector_array2_t cachedStateTrajectoriesStock_;
  vector_array2_t cachedInputTrajectoriesStock_;

  std::vector<std::vector<ModelDataBase>> cachedModelDataTrajectoriesStock_;
  std::vector<std::vector<ModelDataBase>> cachedModelDataEventTimesStock_;
  std::vector<std::vector<ModelDataBase>> cachedProjectedModelDataTrajectoriesStock_;
  std::vector<std::vector<riccati_modification::Data>> cachedRiccatiModificationTrajectoriesStock_;

  scalar_array_t sFinalStock_;
  vector_array_t SvFinalStock_;
  matrix_array_t SmFinalStock_;
  vector_array_t xFinalStock_;

  ScalarFunctionQuadraticApproximation heuristics_;

  ConstraintPenaltyCoefficients constraintPenaltyCoefficients_;

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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Data_T, class Alloc>
void GaussNewtonDDP::correctcachedTrajectoryTail(std::pair<int, scalar_t> timeSegment, const std::vector<Data_T, Alloc>& currentTrajectory,
                                                 std::vector<Data_T, Alloc>& cachedTrajectory) {
  // adding the fist cashed value
  Data_T firstCachedValue;
  LinearInterpolation::interpolate(timeSegment, firstCachedValue, &currentTrajectory);
  cachedTrajectory.emplace_back(firstCachedValue);

  // Concatenate the rest
  const int ignoredSizeOfNominal = timeSegment.first + 1;

  cachedTrajectory.insert(cachedTrajectory.end(), currentTrajectory.begin() + ignoredSizeOfNominal, currentTrajectory.end());
}

}  // namespace ocs2
