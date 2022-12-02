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
#include <ocs2_core/model_data/Metrics.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/oc_data/ProblemMetrics.h>

#include "ocs2_ddp/search_strategy/StrategySettings.h"

namespace ocs2 {

// forward declaration
namespace search_strategy {
struct Solution;
struct SolutionRef;
}  // namespace search_strategy

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
   * @param [in] timePeriod: Initial and final times pair.
   * @param [in] initState: Initial state
   * @param [in] expectedCost: The expected cost based on the LQ model optimization.
   * @param [in] unoptimizedController: The unoptimized controller which search will be performed.
   * @param [in] dualSolution: The dual solution.
   * @param [in] ModeSchedule The current mode schedule.
   * @param [in/out]
   * @param [out] solution: Output of search (primalSolution, performanceIndex, problemMetrics, avgTimeStep)
   * @return whether the search was successful or failed.
   */
  virtual bool run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState, const scalar_t expectedCost,
                   const LinearController& unoptimizedController, const DualSolution& dualSolution, const ModeSchedule& modeSchedule,
                   search_strategy::SolutionRef solution) = 0;

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

 protected:
  const search_strategy::Settings baseSettings_;
};

}  // namespace ocs2

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
namespace ocs2 {
namespace search_strategy {

struct Solution {
  scalar_t avgTimeStep;
  DualSolution dualSolution;
  PrimalSolution primalSolution;
  ProblemMetrics problemMetrics;
  PerformanceIndex performanceIndex;
};

struct SolutionRef {
  SolutionRef(Solution& s)
      : avgTimeStep(s.avgTimeStep),
        dualSolution(s.dualSolution),
        primalSolution(s.primalSolution),
        problemMetrics(s.problemMetrics),
        performanceIndex(s.performanceIndex) {}

  SolutionRef(scalar_t& avgTimeStepArg, DualSolution& dualSolutionArg, PrimalSolution& primalSolutionArg, ProblemMetrics& problemMetricsArg,
              PerformanceIndex& performanceIndexArg)
      : avgTimeStep(avgTimeStepArg),
        dualSolution(dualSolutionArg),
        primalSolution(primalSolutionArg),
        problemMetrics(problemMetricsArg),
        performanceIndex(performanceIndexArg) {}

  scalar_t& avgTimeStep;
  DualSolution& dualSolution;
  PrimalSolution& primalSolution;
  ProblemMetrics& problemMetrics;
  PerformanceIndex& performanceIndex;
};

inline void swap(SolutionRef lhs, SolutionRef rhs) {
  std::swap(lhs.avgTimeStep, rhs.avgTimeStep);
  lhs.dualSolution.swap(rhs.dualSolution);
  lhs.primalSolution.swap(rhs.primalSolution);
  lhs.problemMetrics.swap(rhs.problemMetrics);
  ocs2::swap(lhs.performanceIndex, rhs.performanceIndex);
}

}  // namespace search_strategy
}  // namespace ocs2
