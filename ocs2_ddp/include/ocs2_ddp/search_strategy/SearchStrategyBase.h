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

#include <functional>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>

#include "ocs2_oc/oc_solver/PerformanceIndex.h"
#include "ocs2_oc/rollout/RolloutBase.h"

#include "StrategySettings.h"

namespace ocs2 {

/**
 * This class is an interface class for search strategies such as line-search, trust-region.
 */
class SearchStrategyBase {
 public:
  /**
   * Constructor.
   * @param [in] baseSettings: The basic settings for the search strategy algorithms.
   */
  explicit SearchStrategyBase(search_strategy::Settings baseSettings) : baseSettings_(std::move(baseSettings)) {}

  /**
   * Default destructor.
   */
  virtual ~SearchStrategyBase() = default;

  SearchStrategyBase(const SearchStrategyBase&) = delete;
  SearchStrategyBase& operator=(const SearchStrategyBase&) = delete;

  /**
   * Initializes the strategy.
   *
   * @param [in] initTime: The initial time.
   * @param [in] initState: The initial state.
   * @param [in] finalTime: The final time.
   * @param [in] partitioningTimes: The partitioning times between subsystems.
   * @param [in] initActivePartition: The first active time partition based on the initTime.
   * @param [in] finalActivePartition: The last active time partition based on the finalTime.
   */
  void initalize(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const scalar_array_t& partitioningTimes,
                 size_t initActivePartition, size_t finalActivePartition);

  /**
   * Resets the class to its state after construction.
   */
  virtual void reset() = 0;

  /**
   * Finds the optimal trajectories, controller, and performance index based on the given controller and its increment.
   *
   * @param [in] expectedCost: The expected cost based on the LQ model optimization.
   * @param [in] modeSchedule: The mode schedule.
   * @param [in,out] controllersStock: Array of control policies.
   * @param [in, out] performanceIndex: The current performanceIndex which will be updated to the optimal one.
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] postEventIndicesStock: Array of the post-event indices.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   * @param [out] avgTimeStepFP: The average time-step used during forward rollout.
   * @return whether the search was successful or failed.
   */
  virtual bool run(scalar_t expectedCost, const ModeSchedule& modeSchedule, std::vector<LinearController>& controllersStock,
                   PerformanceIndex& performanceIndex, scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
                   vector_array2_t& stateTrajectoriesStock, vector_array2_t& inputTrajectoriesStock,
                   std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                   std::vector<std::vector<ModelData>>& modelDataEventTimesStock, scalar_t& avgTimeStepFP) = 0;

  /**
   * Checks convergence of the main loop of DDP.
   *
   * @param [in] unreliableControllerIncrement: True if the controller is designed based on an unreliable LQ approximation such as
   * operating trajectories
   * @param [in] previousPerformanceIndex: The previous iteration's PerformanceIndex.
   * @param [in] currentPerformanceIndex: The current iteration's PerformanceIndex.
   * @return A pair of (isOptimizationConverged, infoString)
   */
  virtual std::pair<bool, std::string> checkConvergence(bool unreliableControllerIncrement,
                                                        const PerformanceIndex& previousPerformanceIndex,
                                                        const PerformanceIndex& currentPerformanceIndex) const = 0;

  /**
   * Computes the Riccati modification based on the strategy.
   *
   * @param [in] projectedModelData: The projected data model
   * @param [out] deltaQm: The Riccati modifier to cost 2nd derivative w.r.t. state.
   * @param [out] deltaGv: The Riccati modifier to cost derivative w.r.t. input.
   * @param [out] deltaGm: The Riccati modifier to cost input-state derivative.
   */
  virtual void computeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                          matrix_t& deltaGm) const = 0;

  /**
   * Augments the Hessian of Hamiltonian based on the strategy.
   *
   * @param [in] modelData: The model data.
   * @param [in] Hm: The Hessian of Hamiltonian that should be augmented.
   * @return The augmented Hamiltonian's Hessian.
   */
  virtual matrix_t augmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const = 0;

  /**
   * Evaluates cost and constraints along the given time trajectories.
   *
   * @param [in] constraints: A reference to the constraint.
   * @param [in] costFunction: A reference to the cost function.
   * @param [in] heuristicsFunction: A reference to the heuristics function.
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] postEventIndicesStock: Array of the post-event indices.
   * @param [in] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [in] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [in] constraints: A reference to the constraint class.
   * @param [in] costFunction: A reference to the cost class.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   * @param [out] heuristicsValue: The Heuristics function value.
   */
  void rolloutCostAndConstraints(ConstraintBase& constraints, CostFunctionBase& costFunction, CostFunctionBase& heuristicsFunction,
                                 const scalar_array2_t& timeTrajectoriesStock, const size_array2_t& postEventIndicesStock,
                                 const vector_array2_t& stateTrajectoriesStock, const vector_array2_t& inputTrajectoriesStock,
                                 std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                 std::vector<std::vector<ModelData>>& modelDataEventTimesStock, scalar_t& heuristicsValue) const;

  /**
   * Calculates constraints ISE (Integral of Square Error), cost function integral, and the merit function.
   *
   * @param [in] ineqConstrPenalty: A reference to the inequality constraints penalty function.
   * @param [in] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [in] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [in] modelDataEventTimesStock: Array of model data at event times.
   * @param [in] heuristicsValue: The Heuristics function value.
   *
   * @return The cost, merit function and ISEs of constraints for the trajectory.
   */
  PerformanceIndex calculateRolloutPerformanceIndex(const SoftConstraintPenalty& ineqConstrPenalty,
                                                    const scalar_array2_t& timeTrajectoriesStock,
                                                    const std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                                    const std::vector<std::vector<ModelData>>& modelDataEventTimesStock,
                                                    scalar_t heuristicsValue) const;

 protected:
  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] rollout: A reference to the rollout class.
   * @param [in] modeSchedule: The mode schedule
   * @param [in] controllersStock: Array of control policies.
   * @param [out] timeTrajectoriesStock: Array of trajectories containing the output time trajectory stamp.
   * @param [out] postEventIndicesStock: Array of the post-event indices.
   * @param [out] stateTrajectoriesStock: Array of trajectories containing the output state trajectory.
   * @param [out] inputTrajectoriesStock: Array of trajectories containing the output control input trajectory.
   * @param [out] modelDataTrajectoriesStock: Array of trajectories containing the model data trajectory.
   * @param [out] modelDataEventTimesStock: Array of model data at event times.
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(RolloutBase& rollout, const ModeSchedule& modeSchedule, std::vector<LinearController>& controllersStock,
                             scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
                             vector_array2_t& stateTrajectoriesStock, vector_array2_t& inputTrajectoriesStock,
                             std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                             std::vector<std::vector<ModelData>>& modelDataEventTimesStock) const;

  /**
   * Calculates the integral of the squared (IS) norm of the controller update.
   *
   * @param [in] controllersStock: An array of controllers.
   * @retuen The integral of the squared (IS) norm of the controller update.
   */
  scalar_t calculateControllerUpdateIS(const std::vector<LinearController>& controllersStock) const;

  search_strategy::Settings baseSettings_;

  scalar_t initTime_;
  scalar_t finalTime_;
  vector_t initState_;

  size_t initActivePartition_ = 0;
  size_t finalActivePartition_ = 0;
  size_t numPartitions_ = 0;
  scalar_array_t partitioningTimes_;
};

}  // namespace ocs2
