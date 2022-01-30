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
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>
#include <ocs2_core/reference/ModeSchedule.h>

#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include <ocs2_ddp/DDP_Data.h>

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
   * Resets the class to its state after construction.
   */
  virtual void reset() = 0;

  /**
   * Finds the optimal trajectories, controller, and performance index based on the given controller and its increment.
   *
   * @param [in] initTime: Initial time
   * @param [in] initState: Initial state
   * @param [in] finalTime: Final time
   * @param [in] expectedCost: The expected cost based on the LQ model optimization.
   * @param [in] ModeSchedule The current mode schedule.
   * @param [in, out] controller: Control policies.
   * @param [out] performanceIndex: The current performanceIndex which will be updated to the optimal one.
   * @param [out] dstPrimalData: Resulting time, state, input... trajectories. Attention: Controller inside PrimalDataContainer is NOT
   * optimized. Instead, controller parameter holds the optimized controller.
   * @param [out] avgTimeStepFP: The average time-step used during forward rollout.
   * @return whether the search was successful or failed.
   */
  virtual bool run(const scalar_t initTime, const vector_t& initState, const scalar_t finalTime, const scalar_t expectedCost,
                   const ModeSchedule& modeSchedule, LinearController& controller, PerformanceIndex& performanceIndex,
                   PrimalDataContainer& dstPrimalData, scalar_t& avgTimeStepFP) = 0;

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
   * @param [in] problem: A reference to the optimal control problem.
   * @param [in, out] primalData: Primal Data.
   * @param [out] heuristicsValue: The Heuristics function value.
   */
  void rolloutCostAndConstraints(OptimalControlProblem& problem, PrimalDataContainer& primalData, scalar_t& heuristicsValue) const;

  /**
   * Calculates constraints ISE (Integral of Square Error), cost function integral, and the merit function.
   *
   * @param [in] ineqConstrPenalty: A reference to the inequality constraints penalty function.
   * @param [in] primalData: Primal Data.
   * @param [in] heuristicsValue: The Heuristics function value.
   *
   * @return The cost, merit function and ISEs of constraints for the trajectory.
   */
  PerformanceIndex calculateRolloutPerformanceIndex(const MultidimensionalPenalty& ineqConstrPenalty, const PrimalDataContainer& primalData,
                                                    scalar_t heuristicsValue) const;

 protected:
  /**
   * Forward integrate the system dynamics with given controller. It uses the given control policies and initial state,
   * to integrate the system dynamics in time period [initTime, finalTime].
   *
   * @param [in] rollout: A reference to the rollout class.
   * @param [in] modeSchedule: The mode schedule
   * @param [in] controller: Control policies.
   * @param [out] dstPrimalData: Resulting primal data.
   *
   * @return average time step.
   */
  scalar_t rolloutTrajectory(RolloutBase& rollout, const ModeSchedule& modeSchedule, LinearController& controller,
                             PrimalDataContainer& dstPrimalData) const;

  /**
   * Calculates the integral of the squared (IS) norm of the controller update.
   *
   * @param [in] controller: Input controller.
   * @return The integral of the squared (IS) norm of the controller update.
   */
  scalar_t calculateControllerUpdateIS(const LinearController& controller) const;

  const search_strategy::Settings baseSettings_;
  scalar_t initTime_;
  scalar_t finalTime_;
  vector_t initState_;
};

}  // namespace ocs2
