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
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/model_data/ModelData.h>
#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
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
   * @param [in] rolloutRefStock: An array of references to the rollout.
   * @param [in] optimalControlProblemRef: An array of references to the optimal control problem.
   * @param [in] ineqConstrPenaltyRef: A reference to the inequality constraints penalty.
   * @param [in] meritFunc: the merit function which gets the PerformanceIndex and returns the merit function value.
   */
  LineSearchStrategy(search_strategy::Settings baseSettings, line_search::Settings settings, ThreadPool& threadPoolRef,
                     std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock,
                     std::vector<std::reference_wrapper<OptimalControlProblem>> optimalControlProblemRef,
                     SoftConstraintPenalty& ineqConstrPenaltyRef, std::function<scalar_t(const PerformanceIndex&)> meritFunc);

  /**
   * Default destructor.
   */
  ~LineSearchStrategy() override = default;

  LineSearchStrategy(const LineSearchStrategy&) = delete;
  LineSearchStrategy& operator=(const LineSearchStrategy&) = delete;

  void reset() override {}

  bool run(scalar_t expectedCost, const ModeSchedule& modeSchedule, LinearController& controllersStock, PerformanceIndex& performanceIndex,
           scalar_array_t& timeTrajectoriesStock, size_array_t& postEventIndicesStock, vector_array_t& stateTrajectoriesStock,
           vector_array_t& inputTrajectoriesStock, std::vector<ModelData>& modelDataTrajectoriesStock,
           std::vector<ModelData>& modelDataEventTimesStock, scalar_t& avgTimeStepFP) override;

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
    LinearController initControllersStock;

    std::atomic_size_t alphaExpNext{0};
    std::vector<bool> alphaProcessed;
    std::mutex lineSearchResultMutex;

    std::atomic<scalar_t> stepLengthStar{0.0};
    PerformanceIndex* performanceIndexPtrStar;
    LinearController* controllersStockPtrStar;
    scalar_array_t* timeTrajectoriesStockPtrStar;
    size_array_t* postEventIndicesStockPtrStar;
    vector_array_t* stateTrajectoriesStockPtrStar;
    vector_array_t* inputTrajectoriesStockPtrStar;
    std::vector<ModelData>* modelDataTrajectoriesStockPtrStar;
    std::vector<ModelData>* modelDataEventTimesStockPtrStar;
  };

  line_search::Settings settings_;
  LineSearchModule lineSearchModule_;

  ThreadPool& threadPoolRef_;
  std::atomic_size_t nextTaskId_{0};
  mutable std::mutex outputDisplayGuardMutex_;

  std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock_;
  std::vector<std::reference_wrapper<OptimalControlProblem>> optimalControlProblemRefStock_;
  SoftConstraintPenalty& ineqConstrPenaltyRef_;
  std::function<scalar_t(PerformanceIndex)> meritFunc_;

  std::atomic<scalar_t> avgTimeStepFP_{0.0};
};

}  // namespace ocs2
