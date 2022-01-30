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

#include <ocs2_core/integration/TrapezoidalIntegration.h>

#include <ocs2_ddp/HessianCorrection.h>
#include <ocs2_ddp/search_strategy/LevenbergMarquardtStrategy.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LevenbergMarquardtStrategy::LevenbergMarquardtStrategy(search_strategy::Settings baseSettings, levenberg_marquardt::Settings settings,
                                                       RolloutBase& rolloutRef, OptimalControlProblem& optimalControlProblemRef,
                                                       MultidimensionalPenalty& ineqConstrPenaltyRef,
                                                       std::function<scalar_t(const PerformanceIndex&)> meritFunc)
    : SearchStrategyBase(std::move(baseSettings)),
      settings_(std::move(settings)),
      rolloutRef_(rolloutRef),
      optimalControlProblemRef_(optimalControlProblemRef),
      ineqConstrPenaltyRef_(ineqConstrPenaltyRef),
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
bool LevenbergMarquardtStrategy::run(const scalar_t initTime, const vector_t& initState, const scalar_t finalTime,
                                     const scalar_t expectedCost, const ModeSchedule& modeSchedule, LinearController& controller,
                                     PerformanceIndex& performanceIndex, PrimalDataContainer& dstPrimalData, scalar_t& avgTimeStepFP) {
  // initialize time and state variables
  initState_ = initState;
  initTime_ = initTime;
  finalTime_ = finalTime;

  constexpr size_t taskId = 0;
  // previous merit and the expected reduction
  const auto prevMerit = performanceIndex.merit;
  const auto expectedReduction = performanceIndex.merit - expectedCost;

  // do a full step rollout
  const scalar_t stepLength = numerics::almost_eq(expectedReduction, 0.0) ? 0.0 : 1.0;
  for (size_t k = 0; k < controller.size(); k++) {
    controller.biasArray_[k] += stepLength * controller.deltaBiasArray_[k];
  }

  try {
    // perform a rollout
    const auto avgTimeStep = rolloutTrajectory(rolloutRef_, modeSchedule, controller, dstPrimalData);
    scalar_t heuristicsValue = 0.0;
    rolloutCostAndConstraints(optimalControlProblemRef_, dstPrimalData, heuristicsValue);

    // compute average time step of forward rollout
    avgTimeStepFP_ = 0.9 * avgTimeStepFP_ + 0.1 * avgTimeStep;

    performanceIndex = calculateRolloutPerformanceIndex(ineqConstrPenaltyRef_, dstPrimalData, heuristicsValue);

    // calculates rollout merit
    performanceIndex.merit = meritFunc_(performanceIndex);

    // display
    if (baseSettings_.displayInfo) {
      std::stringstream infoDisplay;
      infoDisplay << "    [Thread " << taskId << "] - step length " << stepLength << '\n';
      infoDisplay << std::setw(4) << performanceIndex << "\n\n";
      std::cerr << infoDisplay.str();
    }

  } catch (const std::exception& error) {
    if (baseSettings_.displayInfo) {
      std::cerr << "    [Thread " << taskId << "] rollout with step length " << stepLength << " is terminated: " << error.what() << "\n";
    }
    performanceIndex.merit = std::numeric_limits<scalar_t>::max();
    performanceIndex.totalCost = std::numeric_limits<scalar_t>::max();
  }

  // compute pho (the ratio between actual reduction and predicted reduction)
  const auto actualReduction = prevMerit - performanceIndex.merit;

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
  const scalar_t relCost = std::abs(currentPerformanceIndex.totalCost + currentPerformanceIndex.inequalityConstraintPenalty -
                                    previousPerformanceIndex.totalCost - previousPerformanceIndex.inequalityConstraintPenalty);
  if (levenbergMarquardtModule_.numSuccessiveRejections == 0 && !unreliableControllerIncrement) {
    isCostFunctionConverged = relCost <= baseSettings_.minRelCost;
  }
  const bool isConstraintsSatisfied = currentPerformanceIndex.stateInputEqConstraintISE <= baseSettings_.constraintTolerance;
  const bool isOptimizationConverged = (isCostFunctionConverged) && isConstraintsSatisfied;

  // convergence info
  std::stringstream infoStream;
  if (isOptimizationConverged) {
    infoStream << "The algorithm has successfully terminated as: \n";

    if (isCostFunctionConverged) {
      infoStream << "    * The absolute relative change of cost (i.e., " << relCost << ") has reached to the minimum value ("
                 << baseSettings_.minRelCost << ").\n";
    }

    infoStream << "    * The ISE of state-input equality constraint (i.e., " << currentPerformanceIndex.stateInputEqConstraintISE
               << ") has reached to its minimum value (" << baseSettings_.constraintTolerance << ").";
  }

  return {isOptimizationConverged, infoStream.str()};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LevenbergMarquardtStrategy::computeRiccatiModification(const ModelData& projectedModelData, matrix_t& deltaQm, vector_t& deltaGv,
                                                            matrix_t& deltaGm) const {
  const auto& HvProjected = projectedModelData.dynamicsBias_;
  const auto& AmProjected = projectedModelData.dynamics_.dfdx;
  const auto& BmProjected = projectedModelData.dynamics_.dfdu;

  // deltaQm, deltaRm, deltaPm
  deltaQm.setZero(projectedModelData.stateDim_, projectedModelData.stateDim_);
  deltaGv.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * HvProjected;
  deltaGm.noalias() = levenbergMarquardtModule_.riccatiMultiple * BmProjected.transpose() * AmProjected;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LevenbergMarquardtStrategy::augmentHamiltonianHessian(const ModelData& modelData, const matrix_t& Hm) const {
  matrix_t HmAug = Hm;
  HmAug.noalias() += levenbergMarquardtModule_.riccatiMultiple * modelData.dynamics_.dfdu.transpose() * modelData.dynamics_.dfdu;
  return HmAug;
}

}  // namespace ocs2
