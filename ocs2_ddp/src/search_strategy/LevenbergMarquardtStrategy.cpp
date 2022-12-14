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

#include "ocs2_ddp/search_strategy/LevenbergMarquardtStrategy.h"

#include <iomanip>

#include "ocs2_ddp/DDP_HelperFunctions.h"
#include "ocs2_ddp/HessianCorrection.h"

#include <ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LevenbergMarquardtStrategy::LevenbergMarquardtStrategy(search_strategy::Settings baseSettings, levenberg_marquardt::Settings settings,
                                                       RolloutBase& rolloutRef, OptimalControlProblem& optimalControlProblemRef,
                                                       std::function<scalar_t(const PerformanceIndex&)> meritFunc)
    : SearchStrategyBase(std::move(baseSettings)),
      settings_(std::move(settings)),
      rolloutRef_(rolloutRef),
      optimalControlProblemRef_(optimalControlProblemRef),
      meritFunc_(std::move(meritFunc)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LevenbergMarquardtStrategy::reset() {
  lmModule_ = LevenbergMarquardtModule();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool LevenbergMarquardtStrategy::run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState,
                                     const scalar_t expectedCost, const LinearController& unoptimizedController,
                                     const DualSolution& dualSolution, const ModeSchedule& modeSchedule,
                                     search_strategy::SolutionRef solution) {
  constexpr size_t taskId = 0;

  // previous merit and the expected reduction
  const auto prevMerit = solution.performanceIndex.merit;
  const auto expectedReduction = solution.performanceIndex.merit - expectedCost;

  // stepsize
  const scalar_t stepLength = numerics::almost_eq(expectedReduction, 0.0) ? 0.0 : 1.0;

  try {
    // compute primal solution
    solution.primalSolution.modeSchedule_ = modeSchedule;
    incrementController(stepLength, unoptimizedController, getLinearController(solution.primalSolution));
    solution.avgTimeStep = rolloutTrajectory(rolloutRef_, timePeriod.first, initState, timePeriod.second, solution.primalSolution);

    // adjust dual solution only if it is required
    const DualSolution* adjustedDualSolutionPtr = &dualSolution;
    if (!dualSolution.timeTrajectory.empty()) {
      // trajectory spreading
      constexpr bool debugPrint = false;
      TrajectorySpreading trajectorySpreading(debugPrint);
      const auto status = trajectorySpreading.set(modeSchedule, solution.primalSolution.modeSchedule_, dualSolution.timeTrajectory);
      if (status.willTruncate || status.willPerformTrajectorySpreading) {
        trajectorySpread(trajectorySpreading, dualSolution, tempDualSolution_);
        adjustedDualSolutionPtr = &tempDualSolution_;
      }
    }

    // initialize dual solution
    initializeDualSolution(optimalControlProblemRef_, solution.primalSolution, *adjustedDualSolutionPtr, solution.dualSolution);

    // compute problem metrics
    computeRolloutMetrics(optimalControlProblemRef_, solution.primalSolution, solution.dualSolution, solution.problemMetrics);

    // compute performanceIndex
    solution.performanceIndex = computeRolloutPerformanceIndex(solution.primalSolution.timeTrajectory_, solution.problemMetrics);
    solution.performanceIndex.merit = meritFunc_(solution.performanceIndex);

    // display
    if (baseSettings_.displayInfo) {
      std::stringstream infoDisplay;
      infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
      infoDisplay << std::setw(4) << solution.performanceIndex << "\n\n";
      std::cerr << infoDisplay.str();
    }

  } catch (const std::exception& error) {
    if (baseSettings_.displayInfo) {
      std::cerr << "    [Thread " << taskId << "] rollout with step length " << stepLength << " is terminated: " << error.what() << "\n";
    }
    solution.performanceIndex.merit = std::numeric_limits<scalar_t>::max();
    solution.performanceIndex.cost = std::numeric_limits<scalar_t>::max();
  }

  // compute pho (the ratio between actual reduction and predicted reduction)
  const auto actualReduction = prevMerit - solution.performanceIndex.merit;
  const auto pho = reductionToPredictedReduction(actualReduction, expectedReduction);

  // display
  if (baseSettings_.displayInfo) {
    std::cerr << "Actual Reduction: " << actualReduction << ",   Predicted Reduction: " << expectedReduction << "\n";
  }

  // adjust riccatiMultipleAdaptiveRatio and riccatiMultiple
  if (pho < 0.25) {
    // increase riccatiMultipleAdaptiveRatio
    lmModule_.riccatiMultipleAdaptiveRatio = std::max(1.0, lmModule_.riccatiMultipleAdaptiveRatio) * settings_.riccatiMultipleDefaultRatio;

    // increase riccatiMultiple
    const auto riccatiMultipleTemp = lmModule_.riccatiMultipleAdaptiveRatio * lmModule_.riccatiMultiple;
    lmModule_.riccatiMultiple = std::max(riccatiMultipleTemp, settings_.riccatiMultipleDefaultFactor);

  } else if (pho > 0.75) {
    // decrease riccatiMultipleAdaptiveRatio
    lmModule_.riccatiMultipleAdaptiveRatio = std::min(1.0, lmModule_.riccatiMultipleAdaptiveRatio) / settings_.riccatiMultipleDefaultRatio;

    // decrease riccatiMultiple
    const auto riccatiMultipleTemp = lmModule_.riccatiMultipleAdaptiveRatio * lmModule_.riccatiMultiple;
    lmModule_.riccatiMultiple = (riccatiMultipleTemp > settings_.riccatiMultipleDefaultFactor) ? riccatiMultipleTemp : 0.0;

  } else {
    lmModule_.riccatiMultipleAdaptiveRatio = 1.0;
    // lmModule_.riccatiMultiple will not change.
  }

  // display
  if (baseSettings_.displayInfo) {
    std::stringstream displayInfo;
    if (lmModule_.numSuccessiveRejections == 0) {
      displayInfo << "The step is accepted with pho: " << pho << ". ";
    } else {
      displayInfo << "The step is rejected with pho: " << pho << " (" << lmModule_.numSuccessiveRejections << " out of "
                  << settings_.maxNumSuccessiveRejections << "). ";
    }

    if (numerics::almost_eq(lmModule_.riccatiMultipleAdaptiveRatio, 1.0)) {
      displayInfo << "The Riccati multiple is kept constant: ";
    } else if (lmModule_.riccatiMultipleAdaptiveRatio < 1.0) {
      displayInfo << "The Riccati multiple is decreased to: ";
    } else {
      displayInfo << "The Riccati multiple is increased to: ";
    }
    displayInfo << lmModule_.riccatiMultiple << ", with ratio: " << lmModule_.riccatiMultipleAdaptiveRatio << ".\n";

    std::cerr << displayInfo.str();
  }

  // max accepted number of successive rejections
  if (lmModule_.numSuccessiveRejections > settings_.maxNumSuccessiveRejections) {
    throw std::runtime_error("The maximum number of successive solution rejections has been reached!");
  }

  // accept or reject the step and modify numSuccessiveRejections
  if (pho >= settings_.minAcceptedPho) {
    // accept the solution
    lmModule_.numSuccessiveRejections = 0;
    return true;

  } else {
    // reject the solution
    ++lmModule_.numSuccessiveRejections;
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<bool, std::string> LevenbergMarquardtStrategy::checkConvergence(bool unreliableControllerIncrement,
                                                                          const PerformanceIndex& previousPerformanceIndex,
                                                                          const PerformanceIndex& currentPerformanceIndex) const {
  // loop break variables
  bool isCostFunctionConverged = false;
  const scalar_t currentTotalCost =
      currentPerformanceIndex.cost + currentPerformanceIndex.equalityLagrangian + currentPerformanceIndex.inequalityLagrangian;
  const scalar_t previousTotalCost =
      previousPerformanceIndex.cost + previousPerformanceIndex.equalityLagrangian + previousPerformanceIndex.inequalityLagrangian;
  const scalar_t relCost = std::abs(currentTotalCost - previousTotalCost);
  if (lmModule_.numSuccessiveRejections == 0 && !unreliableControllerIncrement) {
    isCostFunctionConverged = relCost <= baseSettings_.minRelCost;
  }
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
void LevenbergMarquardtStrategy::computeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                                            matrix_t& deltaGm) const {
  const auto& HvProjected = projectedModelData.dynamicsBias;
  const auto& AmProjected = projectedModelData.dynamics.dfdx;
  const auto& BmProjected = projectedModelData.dynamics.dfdu;

  // deltaQm, deltaRm, deltaPm
  deltaQm.setZero(projectedModelData.stateDim, projectedModelData.stateDim);
  deltaGv.noalias() = lmModule_.riccatiMultiple * BmProjected.transpose() * HvProjected;
  deltaGm.noalias() = lmModule_.riccatiMultiple * BmProjected.transpose() * AmProjected;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LevenbergMarquardtStrategy::augmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const {
  matrix_t HmAug = Hm;
  HmAug.noalias() += lmModule_.riccatiMultiple * modelData.dynamics.dfdu.transpose() * modelData.dynamics.dfdu;
  return HmAug;
}

}  // namespace ocs2
