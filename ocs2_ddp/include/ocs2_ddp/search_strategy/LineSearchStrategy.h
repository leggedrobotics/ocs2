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
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/misc/ThreadPool.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/rollout/RolloutBase.h>

#include "SearchStrategyBase.h"
#include "StrategySettings.h"

namespace ocs2 {

/**
 * Line search strategy: The class computes the nominal controller and the nominal trajectories as well the corresponding performance
 * indices. It line-searches on the feedforward parts of the controller and chooses the largest acceptable step-size.
 */
class LineSearchStrategy final : public SearchStrategyBase {
 public:
  /**
   * constructor.
   *
   * @param [in] baseSettings: The basic settings for the search strategy algorithms.
   * @param [in] settings: The line search settings.
   * @param [in] threadPoolRef: A reference to the thread pool instance.
   * @param [in] rolloutRef: An array of references to the rollout class.
   * @param [in] constraintsRef: An array of references to the constraint class.
   * @param [in] heuristicsFunctionsRef: An array of references to the heuristics function.
   * @param [in] ineqConstrPenaltyRef: A reference to the inequality constraints penalty.
   * @param [in] meritFunc: the merit function which gets the PerformanceIndex and returns the merit function value.
   */
  LineSearchStrategy(search_strategy::Settings baseSettings, line_search::Settings settings, ThreadPool& threadPoolRef,
                     std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock,
                     std::vector<std::reference_wrapper<ConstraintBase>> constraintsRefStock,
                     std::vector<std::reference_wrapper<CostFunctionBase>> costFunctionRefStock,
                     std::vector<std::reference_wrapper<CostFunctionBase>> heuristicsFunctionsRefStock,
                     SoftConstraintPenalty& ineqConstrPenaltyRef, std::function<scalar_t(const PerformanceIndex&)> meritFunc);

  /**
   * Default destructor.
   */
  ~LineSearchStrategy() override = default;

  LineSearchStrategy(const LineSearchStrategy&) = delete;
  LineSearchStrategy& operator=(const LineSearchStrategy&) = delete;

  void reset() override {}

  bool run(scalar_t expectedCost, const ModeSchedule& modeSchedule, std::vector<LinearController>& controllersStock,
           PerformanceIndex& performanceIndex, scalar_array2_t& timeTrajectoriesStock, size_array2_t& postEventIndicesStock,
           vector_array2_t& stateTrajectoriesStock, vector_array2_t& inputTrajectoriesStock,
           std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock, std::vector<std::vector<ModelData>>& modelDataEventTimesStock,
           scalar_t& avgTimeStepFP) override;

  std::pair<bool, std::string> checkConvergence(bool unreliableControllerIncrement, const PerformanceIndex& previousPerformanceIndex,
                                                const PerformanceIndex& currentPerformanceIndex) const override;

  void computeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                  matrix_t& deltaGm) const override;

  matrix_t augmentHamiltonianHessian(const ModelData& /*modelData*/, const matrix_t& Hm) const override { return Hm; }

 private:
  /**
   * Defines line search task on a thread with various learning rates and choose the largest acceptable step-size.
   * The class computes the nominal controller and the nominal trajectories as well the corresponding performance indices.
   */
  void lineSearchTask();

  /** Prints to output. */
  void printString(const std::string& text) const;

  struct LineSearchModule {
    scalar_t baselineMerit = 0.0;           // the merit of the rollout for zero learning rate
    scalar_t initControllerUpdateIS = 0.0;  // integral of the squared (IS) norm of the controller update.
    const ModeSchedule* modeSchedulePtr;
    std::vector<LinearController> initControllersStock;

    std::atomic_size_t alphaExpNext{0};
    std::vector<bool> alphaProcessed;
    std::mutex lineSearchResultMutex;

    std::atomic<scalar_t> stepLengthStar{0.0};
    PerformanceIndex* performanceIndexPtrStar;
    std::vector<LinearController>* controllersStockPtrStar;
    scalar_array2_t* timeTrajectoriesStockPtrStar;
    size_array2_t* postEventIndicesStockPtrStar;
    vector_array2_t* stateTrajectoriesStockPtrStar;
    vector_array2_t* inputTrajectoriesStockPtrStar;
    std::vector<std::vector<ModelData>>* modelDataTrajectoriesStockPtrStar;
    std::vector<std::vector<ModelData>>* modelDataEventTimesStockPtrStar;
  };

  line_search::Settings settings_;
  LineSearchModule lineSearchModule_;

  ThreadPool& threadPoolRef_;
  std::atomic_size_t nextTaskId_{0};
  mutable std::mutex outputDisplayGuardMutex_;

  std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock_;
  std::vector<std::reference_wrapper<ConstraintBase>> constraintsRefStock_;
  std::vector<std::reference_wrapper<CostFunctionBase>> costFunctionRefStock_;
  std::vector<std::reference_wrapper<CostFunctionBase>> heuristicsFunctionsRefStock_;
  SoftConstraintPenalty& ineqConstrPenaltyRef_;
  std::function<scalar_t(PerformanceIndex)> meritFunc_;

  std::atomic<scalar_t> avgTimeStepFP_{0.0};
};

}  // namespace ocs2
