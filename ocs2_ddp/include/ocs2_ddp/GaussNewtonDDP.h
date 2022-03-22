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
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_data/Metrics.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_ddp/DDP_Data.h"
#include "ocs2_ddp/DDP_Settings.h"
#include "ocs2_ddp/riccati_equations/RiccatiModification.h"
#include "ocs2_ddp/search_strategy/SearchStrategyBase.h"

namespace ocs2 {

/**
 * This class is an interface class for the Gauss-Newton DDP based methods.
 */
class GaussNewtonDDP : public SolverBase {
 public:
  /**
   * Constructor

   * @param [in] ddpSettings: Structure containing the settings for the Gauss-Newton DDP algorithm.
   * @param [in] rollout: The rollout class used for simulating the system dynamics.
   * @param [in] optimalControlProblem: The optimal control problem formulation.
   * @param [in] initializer: This class initializes the state-input for the time steps that no controller is available.
   */
  GaussNewtonDDP(ddp::Settings ddpSettings, const RolloutBase& rollout, const OptimalControlProblem& optimalControlProblem,
                 const Initializer& initializer);

  /**
   * Destructor.
   */
  ~GaussNewtonDDP() override;

  void reset() override;

  size_t getNumIterations() const override { return totalNumIterations_; }

  scalar_t getFinalTime() const override { return finalTime_; }

  const PerformanceIndex& getPerformanceIndeces() const override { return performanceIndex_; }

  const std::vector<PerformanceIndex>& getIterationsLog() const override { return performanceIndexHistory_; }

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const final;

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    return getValueFunctionImpl(time, state, nominalPrimalData_, dualData_.valueFunctionTrajectory);
  }

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override;

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    return getStateInputEqualityConstraintLagrangianImpl(time, state, nominalPrimalData_, dualData_);
  }

  std::string getBenchmarkingInfo() const override;

  /**
   * Const access to ddp settings
   */
  const ddp::Settings& settings() const { return ddpSettings_; }

 protected:
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
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @param [out] projectedModelData: The projected model data.
   * @param [out] riccatiModification: The Riccati equation modifier.
   */
  void computeProjectionAndRiccatiModification(const ModelData& modelData, const matrix_t& Sm, ModelData& projectedModelData,
                                               riccati_modification::Data& riccatiModification) const;

  /**
   * Computes the Hessian of Hamiltonian based on the search strategy and algorithm.
   *
   * @param [in] modelData: The model data.
   * @param [in] Sm: The Riccati matrix.
   * @return The Hessian matrix of the Hamiltonian.
   */
  virtual matrix_t computeHamiltonianHessian(const ModelData& modelData, const matrix_t& Sm) const = 0;

  /**
   * Calculates an LQ approximate of the optimal control problem for the nodes.
   *
   * @param [in,out] primalData: The primal Data
   */
  virtual void approximateIntermediateLQ(PrimalDataContainer& primalData) = 0;

  /**
   * Calculate controller for the timeIndex by using primal and dual and write the result back to dstController
   *
   * @param [in] timeIndex: The current time index
   * @param [in] primalData: Primal data used to calculate controller
   * @param [in] dualData: Dual data used to calculate controller
   * @param [out] dstController: The destination controller
   */
  virtual void calculateControllerWorker(size_t timeIndex, const PrimalDataContainer& primalData, const DualDataContainer& dualData,
                                         LinearController& dstController) = 0;

  /**
   * Solves Riccati equations.
   *
   * @param [in] finalValueFunction The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   * @return average time step
   */
  virtual scalar_t solveSequentialRiccatiEquations(const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

  /**
   * The implementation for solving Riccati equations for all the partitions.
   *
   * @param [in] finalValueFunction The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   * @return average time step
   */
  scalar_t solveSequentialRiccatiEquationsImpl(const ScalarFunctionQuadraticApproximation& finalValueFunction);

  /**
   * Solves a Riccati equations and type_1 constraints error correction compensation for the partition in the given index.
   *
   * @param [in] workerIndex: Current worker index
   * @param [in] partitionInterval: Current active interval
   * @param [in] finalValueFunction The final Sm(dfdxx), Sv(dfdx), s(f), for Riccati equation.
   */
  virtual void riccatiEquationsWorker(size_t workerIndex, const std::pair<int, int>& partitionInterval,
                                      const ScalarFunctionQuadraticApproximation& finalValueFunction) = 0;

 private:
  /**
   * Get the State Input Equality Constraint Lagrangian Impl object
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalData: Primal Data
   * @param [in] dualData: DualData
   * @return vector_t
   */
  vector_t getStateInputEqualityConstraintLagrangianImpl(scalar_t time, const vector_t& state, const PrimalDataContainer& primalData,
                                                         const DualDataContainer& dualData) const;

  /**
   * Get the Value Function at time(time) from valueFunctionTrajectory. The the gradient od the value function will be corrected by using
   * the hessian together with the difference between the current state and the corresponding state stored in the primalData.
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @param [in] primalData: Primal Data
   * @param [in] valueFunctionTrajectory: Dual Data
   * @return value function
   */
  ScalarFunctionQuadraticApproximation getValueFunctionImpl(
      const scalar_t time, const vector_t& state, const PrimalDataContainer& primalData,
      const std::vector<ScalarFunctionQuadraticApproximation>& valueFunctionTrajectory) const;

  /**
   * @brief Get the Value Function From Cache
   *
   * @param [in] time: Query time
   * @param [in] state: Current state
   * @return ScalarFunctionQuadraticApproximation
   */
  ScalarFunctionQuadraticApproximation getValueFunctionFromCache(scalar_t time, const vector_t& state) const {
    return getValueFunctionImpl(time, state, cachedPrimalData_, cachedDualData_.valueFunctionTrajectory);
  }

  /**
   * Get the Partition Intervals From Time Trajectory. Intervals are defined as [start, end).
   *
   * Pay attention, the rightmost index of the end partition is (..., timeArray.size() - 1) , as the last value function is filled manually.
   * The reason is though we don’t write to the end index, we do have to read it. Adding the last index to the final partition will
   * cause a segmentation fault. There is no trivial method to distinguish the final partition from other partitions because, by design,
   * partitions should be treated equally.
   *
   * Every time point that is equal or larger to the desiredPartitionPoint should be included in that partition. This logic here is the same
   * as the event times.
   *
   * The last time of desiredPartitionPoints is filled manually. There is no round-off error involved. So it is safe to use == for
   * floating-point numbers. The last time point is naturally included by using std::lower_bound.
   *
   * @param [in] timeTrajectory: time trajectory that will be divided
   * @param [in] numWorkers: number of worker i.e. number of partitions
   * @return array of index pairs indicating the start and end of each partition
   */
  std::vector<std::pair<int, int>> getPartitionIntervalsFromTimeTrajectory(const scalar_array_t& timeTrajectory, int numWorkers);

  /**
   * Forward integrate the system dynamics with given controller and operating trajectories. In general, it uses the
   * given control policies and initial state, to integrate the system dynamics in the time period [initTime, finalTime].
   * However, if the provided controller does not cover the period [initTime, finalTime], it extrapolates (zero-order)
   * the controller until the next event time where after it uses the operating trajectories.
   *
   * Attention: Do NOT pass the controllerPtr of the same primalData used for the first parameter to the second parameter, as all
   * member variables(including controller) of primal data will be cleared.
   *
   * @param [out] primalData: primalData
   * @param [in] controller: nominal controller used to rollout (time, state, input...) trajectories
   * @param [in] workerIndex: working thread (default is 0).
   */
  void rolloutInitialTrajectory(PrimalDataContainer& primalData, ControllerBase* controller, size_t workerIndex = 0);

  /**
   * Calculates the controller. This method uses the following variables. The method modifies unoptimizedController_.
   */
  void calculateController();

  /**
   * Display rollout info and scores.
   */
  void printRolloutInfo() const;

  /**
   * Calculates the merit function based on the performance index .
   *
   * @param [in] performanceIndex: The performance index which includes the uninitialized merit, cost, and ISEs of constraints.
   * @return The merit function
   */
  scalar_t calculateRolloutMerit(const PerformanceIndex& performanceIndex) const;

  /**
   * Calculates max feedforward update norm and max type-1 error update norm.
   *
   * @param maxDeltaUffNorm: max feedforward update norm.
   * @param maxDeltaUeeNorm: max type-1 error update norm.
   */

  /**
   * Calculates max feedforward update norm of the controller.
   *
   * @param [in] controller: Control policy
   * @return max feedforward update norm.
   */
  scalar_t maxControllerUpdateNorm(const LinearController& controller) const;

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
   * Projects the unconstrained LQ coefficients to constrained ones.
   *
   * @param [in] modelData: The model data.
   * @param [in] constraintRangeProjector: The projection matrix to the constrained subspace.
   * @param [in] constraintNullProjector: The projection matrix to the null space of constrained.
   * @param [out] projectedModelData: The projected model data.
   */
  void projectLQ(const ModelData& modelData, const matrix_t& constraintRangeProjector, const matrix_t& constraintNullProjector,
                 ModelData& projectedModelData) const;

  /**
   * Initialize the constraint penalty coefficients.
   */
  void initializeConstraintPenalties();

  /**
   * Updates the constraint penalty coefficients.
   * @param [in] equalityConstraintsSSE: SSE of the equality constraints.
   */
  void updateConstraintPenalties(scalar_t equalityConstraintsSSE);

  /**
   * Runs the search strategy. It updates the controller and the corresponding trajectories only when the search is successful.
   * If fail, the cached primal data will be written to dstPrimalData.
   *
   * @param [in] lqModelExpectedCost: The expected cost based on the LQ model optimization.
   * @param [in] unoptimizedController: The unoptimized controller which search will be performed.
   * @param [out] primalData: Optimized primal data container if it is an final search. otherwise nominal data container
   * @param [out] performanceIndex: The optimal performanceIndex which will be updated to the optimal one.
   * @param [out] metrics: The optimal trajectories metrics.
   */
  void runSearchStrategy(scalar_t lqModelExpectedCost, const LinearController& unoptimizedController, PrimalDataContainer& primalData,
                         PerformanceIndex& performanceIndex, MetricsCollection& metrics);

  /**
   * swap both primal and dual data cache
   */
  void swapDataToCache();

  /**
   * Runs the initialization method for Gauss-Newton DDP.
   */
  void runInit();

  /**
   * Runs a single iteration of Gauss-Newton DDP.
   * @param [in] lqModelExpectedCost: The expected cost based on the LQ model optimization.
   */
  void runIteration(scalar_t lqModelExpectedCost);

  /**
   * Checks convergence of the main loop of DDP.
   *
   * @param [in] isInitalControllerEmpty: Whether the initial controller was empty.
   * @param [in] previousPerformanceIndex: The previous iteration's PerformanceIndex.
   * @param [in] currentPerformanceIndex: The current iteration's PerformanceIndex.
   * @return A pair of (isOptimizationConverged, infoString)
   */
  std::pair<bool, std::string> checkConvergence(bool isInitalControllerEmpty, const PerformanceIndex& previousPerformanceIndex,
                                                const PerformanceIndex& currentPerformanceIndex) const;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    runImpl(initTime, initState, finalTime, nullptr);
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override;

 protected:
  PrimalDataContainer nominalPrimalData_, optimizedPrimalData_;
  // controller that is calculated directly from dual solution. It is unoptimized because it haven't gone through searching.
  LinearController unoptimizedController_;

  // multi-threading helper variables
  std::atomic_size_t nextTaskId_{0};
  std::atomic_size_t nextTimeIndex_{0};

  scalar_t initTime_ = 0.0;
  scalar_t finalTime_ = 0.0;
  vector_t initState_;

  std::unique_ptr<SearchStrategyBase> searchStrategyPtr_;
  std::vector<OptimalControlProblem> optimalControlProblemStock_;

  DualDataContainer dualData_;

 private:
  const ddp::Settings ddpSettings_;

  ThreadPool threadPool_;

  unsigned long long int totalNumIterations_{0};

  PerformanceIndex performanceIndex_;

  std::vector<PerformanceIndex> performanceIndexHistory_;

  std::vector<std::unique_ptr<RolloutBase>> dynamicsForwardRolloutPtrStock_;
  std::vector<std::unique_ptr<RolloutBase>> initializerRolloutPtrStock_;

  // used for caching the nominal trajectories for which the LQ problem is
  // constructed and solved before terminating run()
  PrimalDataContainer cachedPrimalData_;
  DualDataContainer cachedDualData_;

  MetricsCollection metrics_;

  ScalarFunctionQuadraticApproximation heuristics_;

  struct ConstraintPenaltyCoefficients {
    scalar_t penaltyTol = 1e-3;
    scalar_t penaltyCoeff = 0.0;
  };
  ConstraintPenaltyCoefficients constraintPenaltyCoefficients_;

  // forward pass and backward pass average time step
  scalar_t avgTimeStepFP_ = 0.0;
  scalar_t avgTimeStepBP_ = 0.0;

  // benchmarking
  benchmark::RepeatedTimer initializationTimer_;
  benchmark::RepeatedTimer linearQuadraticApproximationTimer_;
  benchmark::RepeatedTimer backwardPassTimer_;
  benchmark::RepeatedTimer computeControllerTimer_;
  benchmark::RepeatedTimer searchStrategyTimer_;
};

}  // namespace ocs2
