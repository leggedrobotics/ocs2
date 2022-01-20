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
      rolloutRefStock_(std::move(rolloutRefStock)),
      optimalControlProblemRefStock_(std::move(optimalControlProblemRefStock)),
      meritFunc_(std::move(meritFunc)),
      temporaryMemories_(threadPoolRef.numThreads() + 1) {
  // infeasible learning rate adjustment scheme
  if (!numerics::almost_ge(settings_.maxStepLength_, settings_.minStepLength_)) {
    throw std::runtime_error("The maximum learning rate is smaller than the minimum learning rate.");
  }

  // Initialize controller
  for (auto& temporaryMemory : temporaryMemories_) {
    temporaryMemory.primalSolution.controllerPtr_.reset(new LinearController);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool LineSearchStrategy::run(const scalar_t initTime, const vector_t& initState, const scalar_t finalTime, const scalar_t expectedCost,
                             const LinearController& unoptimizedController, const ModeSchedule& modeSchedule,
                             PrimalSolution& primalSolution, PerformanceIndex& performanceIndex, MetricsCollection& metrics,
                             scalar_t& avgTimeStepFP) {
  // number of line search iterations (the if statements order is important)
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

  // perform a rollout with steplength zero.
  constexpr size_t taskId = 0;
  constexpr scalar_t stepLength = 0.0;
  try {
    // perform a rollout
    incrementController(stepLength, unoptimizedController, getLinearController(primalSolution));
    const auto avgTimeStep = rolloutTrajectory(rolloutRefStock_[taskId], initTime, initState, finalTime, modeSchedule, primalSolution);
    // compute average time step of forward rollout
    avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStep;

    // rollout cost and constraints
    computeRolloutMetrics(optimalControlProblemRefStock_[taskId], primalSolution, metrics);

    performanceIndex = computeRolloutPerformanceIndex(primalSolution.timeTrajectory_, metrics);

    // calculates rollout merit
    performanceIndex.merit = meritFunc_(performanceIndex);

    // display
    if (baseSettings_.displayInfo) {
      std::stringstream infoDisplay;
      infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
      infoDisplay << std::setw(4) << performanceIndex << "\n\n";
      printString(infoDisplay.str());
    }

  } catch (const std::exception& error) {
    if (baseSettings_.displayInfo) {
      printString("    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                  " is terminated: " + error.what() + '\n');
    }
    throw std::runtime_error("DDP controller does not generate a stable rollout.");
  }

  // initialize lineSearchModule
  lineSearchModule_.baselineMerit = performanceIndex.merit;
  lineSearchModule_.initControllerUpdateIS = computeControllerUpdateIS(unoptimizedController);
  lineSearchModule_.modeSchedulePtr = &modeSchedule;
  lineSearchModule_.unoptimizedControllerPtr = &unoptimizedController;

  lineSearchModule_.alphaExpNext = 0;
  lineSearchModule_.alphaProcessed = std::vector<bool>(maxNumOfLineSearches, false);

  lineSearchModule_.stepLengthStar = 0.0;
  lineSearchModule_.performanceIndexStarPtr = &performanceIndex;
  lineSearchModule_.primalSolutionStarPtr = &primalSolution;
  lineSearchModule_.metricsStarPtr = &metrics;

  nextTaskId_ = 0;
  auto task = [&](int) { lineSearchTask(initTime, initState, finalTime); };
  threadPoolRef_.runParallel(task, threadPoolRef_.numThreads());

  // revitalize all integrators
  for (RolloutBase& rollout : rolloutRefStock_) {
    rollout.reactivateRollout();
  }

  avgTimeStepFP = avgTimeStepFP_;

  // display
  if (baseSettings_.displayInfo) {
    std::cerr << "The chosen step length is: " + std::to_string(lineSearchModule_.stepLengthStar) << "\n";
  }

  return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LineSearchStrategy::lineSearchTask(const scalar_t initTime, const vector_t& initState, const scalar_t finalTime) {
  // assign task ID (atomic)
  size_t taskId = nextTaskId_++;

  auto& problem = optimalControlProblemRefStock_[taskId];
  auto& temporaryMemory = temporaryMemories_[taskId];
  auto& performanceIndex = temporaryMemory.performanceIndex;
  auto& primalSolution = temporaryMemory.primalSolution;
  auto& metrics = temporaryMemory.metrics;

  while (true) {
    size_t alphaExp = lineSearchModule_.alphaExpNext++;
    scalar_t stepLength = settings_.maxStepLength_ * std::pow(settings_.contractionRate_, alphaExp);

    /*
     * finish this thread's task since the learning rate is less than the minimum learning rate.
     * This means that the all the line search tasks are already processed or they are under
     * process in other threads.
     */
    if (!numerics::almost_ge(stepLength, settings_.minStepLength_)) {
      break;
    }

    // skip if the current learning rate is less than the best candidate
    if (stepLength < lineSearchModule_.stepLengthStar) {
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
      // perform a rollout
      incrementController(stepLength, *lineSearchModule_.unoptimizedControllerPtr, getLinearController(primalSolution));
      const auto avgTimeStep =
          rolloutTrajectory(rolloutRefStock_[taskId], initTime, initState, finalTime, *lineSearchModule_.modeSchedulePtr, primalSolution);
      // compute average time step of forward rollout
      avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStep;

      // rollout cost and constraints
      computeRolloutMetrics(problem, primalSolution, metrics);

      performanceIndex = computeRolloutPerformanceIndex(primalSolution.timeTrajectory_, metrics);

      // calculates rollout merit
      performanceIndex.merit = meritFunc_(performanceIndex);

      // display
      if (baseSettings_.displayInfo) {
        std::stringstream infoDisplay;
        infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
        infoDisplay << std::setw(4) << performanceIndex << "\n\n";
        printString(infoDisplay.str());
      }

    } catch (const std::exception& error) {
      if (baseSettings_.displayInfo) {
        printString("    [Thread " + std::to_string(taskId) + "] rollout with step length " + std::to_string(stepLength) +
                    " is terminated: " + error.what() + '\n');
      }
      performanceIndex.merit = std::numeric_limits<scalar_t>::max();
      performanceIndex.totalCost = std::numeric_limits<scalar_t>::max();
    }

    // whether to accept the step or reject it
    bool terminateLinesearchTasks = false;
    {
      std::lock_guard<std::mutex> lock(lineSearchModule_.lineSearchResultMutex);

      /*
       * based on the "Armijo backtracking" step length selection policy:
       * cost should be better than the baseline cost but learning rate should
       * be as high as possible. This is equivalent to a single core line search.
       */
      const bool progressCondition = performanceIndex.merit < (lineSearchModule_.baselineMerit * (1.0 - 1e-3 * stepLength));
      const bool armijoCondition =
          performanceIndex.merit <
          (lineSearchModule_.baselineMerit - settings_.armijoCoefficient_ * stepLength * lineSearchModule_.initControllerUpdateIS);
      if (armijoCondition && stepLength > lineSearchModule_.stepLengthStar) {
        lineSearchModule_.stepLengthStar = stepLength;
        *lineSearchModule_.performanceIndexStarPtr = performanceIndex;
        lineSearchModule_.primalSolutionStarPtr->swap(primalSolution);
        swap(*lineSearchModule_.metricsStarPtr, metrics);

        // whether to stop all other thread.
        terminateLinesearchTasks = true;
        for (size_t i = 0; i < alphaExp; i++) {
          if (!lineSearchModule_.alphaProcessed[i]) {
            terminateLinesearchTasks = false;
            break;
          }
        }  // end of i loop

      }  // end of if

      lineSearchModule_.alphaProcessed[alphaExp] = true;

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
  bool isStepLengthStarZero = false;
  bool isCostFunctionConverged = false;
  const scalar_t currentTotalCost = currentPerformanceIndex.totalCost + currentPerformanceIndex.equalityLagrangiansPenalty +
                                    currentPerformanceIndex.inequalityLagrangiansPenalty;
  const scalar_t previousTotalCost = previousPerformanceIndex.totalCost + previousPerformanceIndex.equalityLagrangiansPenalty +
                                     previousPerformanceIndex.inequalityLagrangiansPenalty;
  const scalar_t relCost = std::abs(currentTotalCost - previousTotalCost);
  isStepLengthStarZero = numerics::almost_eq(lineSearchModule_.stepLengthStar.load(), 0.0) && !unreliableControllerIncrement;
  isCostFunctionConverged = relCost <= baseSettings_.minRelCost;
  const bool isConstraintsSatisfied = currentPerformanceIndex.equalityConstraintsSSE <= baseSettings_.constraintTolerance;
  const bool isOptimizationConverged = (isCostFunctionConverged || isStepLengthStarZero) && isConstraintsSatisfied;

  // convergence info
  std::stringstream infoStream;
  if (isOptimizationConverged) {
    infoStream << "The algorithm has successfully terminated as: \n";

    if (isStepLengthStarZero) {
      infoStream << "    * The step length reduced to zero.\n";
    }

    if (isCostFunctionConverged) {
      infoStream << "    * The absolute relative change of cost (i.e., " << relCost << ") has reached to the minimum value ("
                 << baseSettings_.minRelCost << ").\n";
    }

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
