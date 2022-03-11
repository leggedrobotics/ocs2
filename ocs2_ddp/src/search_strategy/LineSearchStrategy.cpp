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

#include "ocs2_ddp/search_strategy/LineSearchStrategy.h"

#include "ocs2_ddp/DDP_HelperFunctions.h"
#include "ocs2_ddp/HessianCorrection.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LineSearchStrategy::LineSearchStrategy(search_strategy::Settings baseSettings, line_search::Settings settings, ThreadPool& threadPoolRef,
                                       std::vector<std::reference_wrapper<RolloutBase>> rolloutRefStock,
                                       std::vector<std::reference_wrapper<OptimalControlProblem>> optimalControlProblemRefStock,
                                       std::function<scalar_t(const PerformanceIndex&)> meritFunc)
    : SearchStrategyBase(std::move(baseSettings)),
      settings_(std::move(settings)),
      threadPoolRef_(threadPoolRef),
      workersSolution_(threadPoolRef.numThreads() + 1),
      rolloutRefStock_(std::move(rolloutRefStock)),
      optimalControlProblemRefStock_(std::move(optimalControlProblemRefStock)),
      meritFunc_(std::move(meritFunc)) {
  // infeasible learning rate adjustment scheme
  if (!numerics::almost_ge(settings_.maxStepLength_, settings_.minStepLength_)) {
    throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");
  }

  // Initialize controller
  for (auto& solution : workersSolution_) {
    solution.primalSolution.controllerPtr_.reset(new LinearController);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LineSearchStrategy::maxNumOfSearches() const {
  size_t maxNumOfLineSearches = 0;
  if (numerics::almost_eq(settings_.minStepLength_, settings_.maxStepLength_)) {
    maxNumOfLineSearches = 1;
  } else if (settings_.maxStepLength_ < settings_.minStepLength_) {
    maxNumOfLineSearches = 0;
  } else {
    const auto ratio = settings_.minStepLength_ / settings_.maxStepLength_;
    maxNumOfLineSearches =
        static_cast<size_t>(std::log(ratio + numeric_traits::limitEpsilon<scalar_t>()) / std::log(settings_.contractionRate_) + 1);
  }
  return maxNumOfLineSearches;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LineSearchStrategy::computeSolution(size_t taskId, scalar_t stepLength, search_strategy::Solution& solution) {
  auto& problem = optimalControlProblemRefStock_[taskId];
  auto& rollout = rolloutRefStock_[taskId];

  // compute primal solution
  solution.primalSolution.modeSchedule_ = *lineSearchInputRef_.modeSchedulePtr;
  incrementController(stepLength, *lineSearchInputRef_.unoptimizedControllerPtr, getLinearController(solution.primalSolution));
  solution.avgTimeStep = rolloutTrajectory(rollout, lineSearchInputRef_.timePeriodPtr->first, *lineSearchInputRef_.initStatePtr,
                                           lineSearchInputRef_.timePeriodPtr->second, solution.primalSolution);

  // compute metrics
  computeRolloutMetrics(problem, solution.primalSolution, solution.metrics);

  // compute performanceIndex
  solution.performanceIndex = computeRolloutPerformanceIndex(solution.primalSolution.timeTrajectory_, solution.metrics);
  solution.performanceIndex.merit = meritFunc_(solution.performanceIndex);

  // display
  if (baseSettings_.displayInfo) {
    std::stringstream infoDisplay;
    infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
    infoDisplay << std::setw(4) << solution.performanceIndex << "\n\n";
    printString(infoDisplay.str());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool LineSearchStrategy::run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState, const scalar_t expectedCost,
                             const LinearController& unoptimizedController, const ModeSchedule& modeSchedule,
                             search_strategy::SolutionRef solutionRef) {
  // initialize lineSearchModule inputs
  lineSearchInputRef_.timePeriodPtr = &timePeriod;
  lineSearchInputRef_.initStatePtr = &initState;
  lineSearchInputRef_.unoptimizedControllerPtr = &unoptimizedController;
  lineSearchInputRef_.modeSchedulePtr = &modeSchedule;
  bestSolutionRef_ = &solutionRef;

  // perform a rollout with steplength zero.
  constexpr size_t taskId = 0;
  constexpr scalar_t stepLength = 0.0;
  try {
    computeSolution(taskId, stepLength, workersSolution_[taskId]);
    baselineMerit_ = workersSolution_[taskId].performanceIndex.merit;
    unoptimizedControllerUpdateIS_ = computeControllerUpdateIS(unoptimizedController);

    // record solution
    bestStepSize_ = stepLength;
    swap(*bestSolutionRef_, workersSolution_[taskId]);

  } catch (const std::exception& error) {
    if (baseSettings_.displayInfo) {
      printString("    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                  " is terminated: " + error.what() + '\n');
    }
    throw std::runtime_error("[SearchStrategy::run] DDP controller does not generate a stable rollout!");
  }

  // run workers
  nextTaskId_ = 0;
  alphaExpNext_ = 0;
  alphaProcessed_ = std::vector<bool>(maxNumOfSearches(), false);
  auto task = [&](int) { lineSearchTask(nextTaskId_++); };
  threadPoolRef_.runParallel(task, threadPoolRef_.numThreads());

  // revitalize all integrators
  for (RolloutBase& rollout : rolloutRefStock_) {
    rollout.reactivateRollout();
  }

  // display
  if (baseSettings_.displayInfo) {
    std::cerr << "The chosen step length is: " + std::to_string(bestStepSize_) << "\n";
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LineSearchStrategy::lineSearchTask(const size_t taskId) {
  while (true) {
    const size_t alphaExp = alphaExpNext_++;
    const scalar_t stepLength = settings_.maxStepLength_ * std::pow(settings_.contractionRate_, alphaExp);

    /*
     * finish this thread's task since the learning rate is less than the minimum learning rate.
     * This means that the all the line search tasks are already processed or they are under
     * process in other threads.
     */
    if (!numerics::almost_ge(stepLength, settings_.minStepLength_)) {
      break;
    }

    // skip if the current learning rate is less than the best candidate
    if (stepLength < bestStepSize_) {
      // display
      if (baseSettings_.displayInfo) {
        std::string linesearchDisplay;
        linesearchDisplay = "    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                            " is skipped: A larger learning rate is already found!\n";
        printString(linesearchDisplay);
      }
      break;
    }

    try {
      computeSolution(taskId, stepLength, workersSolution_[taskId]);
    } catch (const std::exception& error) {
      if (baseSettings_.displayInfo) {
        printString("    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                    " is terminated: " + error.what() + '\n');
      }
      workersSolution_[taskId].performanceIndex.merit = std::numeric_limits<scalar_t>::max();
      workersSolution_[taskId].performanceIndex.cost = std::numeric_limits<scalar_t>::max();
    }

    // whether to accept the step or reject it
    bool terminateLinesearchTasks = false;
    {
      std::lock_guard<std::mutex> lock(lineSearchResultMutex_);

      /*
       * based on the "Armijo backtracking" step length selection policy:
       * cost should be better than the baseline cost but learning rate should
       * be as high as possible. This is equivalent to a single core line search.
       */
      const bool progressCondition = workersSolution_[taskId].performanceIndex.merit < (baselineMerit_ * (1.0 - 1e-3 * stepLength));
      const bool armijoCondition = workersSolution_[taskId].performanceIndex.merit <
                                   (baselineMerit_ - settings_.armijoCoefficient_ * stepLength * unoptimizedControllerUpdateIS_);

      if (armijoCondition && stepLength > bestStepSize_) {
        bestStepSize_ = stepLength;
        swap(*bestSolutionRef_, workersSolution_[taskId]);

        // whether to stop all other thread.
        terminateLinesearchTasks = true;
        for (size_t i = 0; i < alphaExp; i++) {
          if (!alphaProcessed_[i]) {
            terminateLinesearchTasks = false;
            break;
          }
        }  // end of i loop

      }  // end of if

      alphaProcessed_[alphaExp] = true;

    }  // end lock

    // kill other ongoing line search tasks
    if (terminateLinesearchTasks) {
      for (RolloutBase& rollout : rolloutRefStock_) {
        rollout.abortRollout();
      }
      if (baseSettings_.displayInfo) {
        printString("    LS: interrupt other rollout's integrations.\n");
      }
      break;
    }

  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, std::string> LineSearchStrategy::checkConvergence(bool unreliableControllerIncrement,
                                                                  const PerformanceIndex& previousPerformanceIndex,
                                                                  const PerformanceIndex& currentPerformanceIndex) const {
  // loop break variables
  const scalar_t currentTotalCost =
      currentPerformanceIndex.cost + currentPerformanceIndex.equalityLagrangian + currentPerformanceIndex.inequalityLagrangian;
  const scalar_t previousTotalCost =
      previousPerformanceIndex.cost + previousPerformanceIndex.equalityLagrangian + previousPerformanceIndex.inequalityLagrangian;
  const scalar_t relCost = std::abs(currentTotalCost - previousTotalCost);
  const bool isCostFunctionConverged = relCost <= baseSettings_.minRelCost;
  const bool isConstraintsSatisfied = currentPerformanceIndex.equalityConstraintsSSE <= baseSettings_.constraintTolerance;
  const bool isOptimizationConverged = isCostFunctionConverged && isConstraintsSatisfied;

  // convergence info
  std::stringstream infoStream;
  if (isOptimizationConverged) {
    infoStream << "The algorithm has successfully terminated as: \n";

    infoStream << "    * The absolute relative change of cost (i.e., " << relCost << ") has reached to the minimum value ("
               << baseSettings_.minRelCost << ").\n";

    infoStream << "    * The SSE of equality constraints (i.e., " << currentPerformanceIndex.equalityConstraintsSSE
               << ") has reached to its minimum value (" << baseSettings_.constraintTolerance << ").";
  }

  return {isOptimizationConverged, infoStream.str()};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LineSearchStrategy::computeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                                    matrix_t& deltaGm) const {
  const auto& QmProjected = projectedModelData.cost.dfdxx;
  const auto& PmProjected = projectedModelData.cost.dfdux;

  // Q_minus_PTRinvP
  matrix_t Q_minus_PTRinvP = QmProjected;
  Q_minus_PTRinvP.noalias() -= PmProjected.transpose() * PmProjected;

  // deltaQm
  deltaQm = Q_minus_PTRinvP;
  hessian_correction::shiftHessian(settings_.hessianCorrectionStrategy_, deltaQm, settings_.hessianCorrectionMultiple_);
  deltaQm -= Q_minus_PTRinvP;

  // deltaGv, deltaGm
  const auto projectedInputDim = projectedModelData.dynamics.dfdu.cols();
  deltaGv.setZero(projectedInputDim, 1);
  deltaGm.setZero(projectedInputDim, projectedModelData.stateDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LineSearchStrategy::printString(const std::string& text) const {
  std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
  std::cerr << text;
}

}  // namespace ocs2
