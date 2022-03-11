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

#include "ocs2_ddp/DDP_HelperFunctions.h"
#include "ocs2_ddp/HessianCorrection.h"

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
  levenbergMarquardtModule_ = LevenbergMarquardtModule();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool LevenbergMarquardtStrategy::run(const std::pair<scalar_t, scalar_t>& timePeriod, const vector_t& initState,
                                     const scalar_t expectedCost, const LinearController& unoptimizedController,
                                     const ModeSchedule& modeSchedule, search_strategy::SolutionRef solution) {
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

    // compute metrics
    computeRolloutMetrics(optimalControlProblemRef_, solution.primalSolution, solution.metrics);

    // compute performanceIndex
    solution.performanceIndex = computeRolloutPerformanceIndex(solution.primalSolution.timeTrajectory_, solution.metrics);
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

  if (std::abs(actualReduction) < baseSettings_.minRelCost || expectedReduction <= baseSettings_.minRelCost) {
    levenbergMarquardtModule_.pho = 1.0;
  } else if (actualReduction < 0.0) {
    levenbergMarquardtModule_.pho = 0.0;
  } else {
    levenbergMarquardtModule_.pho = actualReduction / expectedReduction;
  }

  // display
  if (baseSettings_.displayInfo) {
    std::cerr << "Actual Reduction: " << actualReduction << ",   Predicted Reduction: " << expectedReduction << "\n";
  }

  // adjust riccatiMultipleAdaptiveRatio and riccatiMultiple
  if (levenbergMarquardtModule_.pho < 0.25) {
    // increase riccatiMultipleAdaptiveRatio
    levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio =
        std::max(1.0, levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio) * settings_.riccatiMultipleDefaultRatio_;

    // increase riccatiMultiple
    auto riccatiMultipleTemp = levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio * levenbergMarquardtModule_.riccatiMultiple;
    if (riccatiMultipleTemp > settings_.riccatiMultipleDefaultFactor_) {
      levenbergMarquardtModule_.riccatiMultiple = riccatiMultipleTemp;
    } else {
      levenbergMarquardtModule_.riccatiMultiple = settings_.riccatiMultipleDefaultFactor_;
    }

  } else if (levenbergMarquardtModule_.pho > 0.75) {
    // decrease riccatiMultipleAdaptiveRatio
    levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio =
        std::min(1.0, levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio) / settings_.riccatiMultipleDefaultRatio_;

    // decrease riccatiMultiple
    auto riccatiMultipleTemp = levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio * levenbergMarquardtModule_.riccatiMultiple;
    if (riccatiMultipleTemp > settings_.riccatiMultipleDefaultFactor_) {
      levenbergMarquardtModule_.riccatiMultiple = riccatiMultipleTemp;
    } else {
      levenbergMarquardtModule_.riccatiMultiple = 0.0;
    }
  } else {
    levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio = 1.0;
    // levenbergMarquardtModule_.riccatiMultiple will not change.
  }

  // display
  if (baseSettings_.displayInfo) {
    std::stringstream displayInfo;
    if (levenbergMarquardtModule_.numSuccessiveRejections == 0) {
      displayInfo << "The step is accepted with pho: " << levenbergMarquardtModule_.pho << ". ";
    } else {
      displayInfo << "The step is rejected with pho: " << levenbergMarquardtModule_.pho << " ("
                  << levenbergMarquardtModule_.numSuccessiveRejections << " out of " << settings_.maxNumSuccessiveRejections_ << "). ";
    }

    if (numerics::almost_eq(levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio, 1.0)) {
      displayInfo << "The Riccati multiple is kept constant: ";
    } else if (levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio < 1.0) {
      displayInfo << "The Riccati multiple is decreased to: ";
    } else {
      displayInfo << "The Riccati multiple is increased to: ";
    }
    displayInfo << levenbergMarquardtModule_.riccatiMultiple << ", with ratio: " << levenbergMarquardtModule_.riccatiMultipleAdaptiveRatio
                << ".\n";

    std::cerr << displayInfo.str();
  }

  // max accepted number of successive rejections
  if (levenbergMarquardtModule_.numSuccessiveRejections > settings_.maxNumSuccessiveRejections_) {
    throw std::runtime_error("The maximum number of successive solution rejections has been reached!");
  }

  // accept or reject the step and modify numSuccessiveRejections
  if (levenbergMarquardtModule_.pho >= settings_.minAcceptedPho_) {
    // accept the solution
    levenbergMarquardtModule_.numSuccessiveRejections = 0;
    return true;

  } else {
    // reject the solution
    ++levenbergMarquardtModule_.numSuccessiveRejections;
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
  if (levenbergMarquardtModule_.numSuccessiveRejections == 0 && !unreliableControllerIncrement) {
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
  deltaGv.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * HvProjected;
  deltaGm.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * AmProjected;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LevenbergMarquardtStrategy::augmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const {
  matrix_t HmAug = Hm;
  HmAug.noalias() += levenbergMarquardtModule_.riccatiMultiple * modelData.dynamics.dfdu.transpose() * modelData.dynamics.dfdu;
  return HmAug;
}

}  // namespace ocs2
