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

#include "ocs2_ddp/DDP_HelperFunctions.h"

#include <algorithm>
#include <iostream>

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/integration/TrapezoidalIntegration.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/approximate_model/ChangeOfInputVariables.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

namespace ocs2 {

namespace {
template <typename DataType>
void copySegment(const LinearInterpolation::index_alpha_t& indexAlpha0, const LinearInterpolation::index_alpha_t& indexAlpha1,
                 const std::vector<DataType>& inputTrajectory, std::vector<DataType>& outputTrajectory) {
  outputTrajectory.clear();
  outputTrajectory.resize(2 + indexAlpha1.first - indexAlpha0.first);

  if (!outputTrajectory.empty()) {
    outputTrajectory.front() = LinearInterpolation::interpolate(indexAlpha0, inputTrajectory);
    if (indexAlpha1.first >= indexAlpha0.first) {
      std::copy(inputTrajectory.begin() + indexAlpha0.first + 1, inputTrajectory.begin() + indexAlpha1.first + 1,
                outputTrajectory.begin() + 1);
    }
    outputTrajectory.back() = LinearInterpolation::interpolate(indexAlpha1, inputTrajectory);
  }
}
}  // unnamed namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void computeRolloutMetrics(OptimalControlProblem& problem, const PrimalSolution& primalSolution, DualSolutionConstRef dualSolution,
                           ProblemMetrics& problemMetrics) {
  const auto& tTrajectory = primalSolution.timeTrajectory_;
  const auto& xTrajectory = primalSolution.stateTrajectory_;
  const auto& uTrajectory = primalSolution.inputTrajectory_;
  const auto& postEventIndices = primalSolution.postEventIndices_;

  problemMetrics.clear();
  problemMetrics.preJumps.reserve(postEventIndices.size());
  problemMetrics.intermediates.reserve(tTrajectory.size());

  auto nextPostEventIndexItr = postEventIndices.begin();
  constexpr auto request = Request::Cost + Request::Constraint + Request::SoftConstraint;
  for (size_t k = 0; k < tTrajectory.size(); k++) {
    // intermediate time cost and constraints
    problem.preComputationPtr->request(request, tTrajectory[k], xTrajectory[k], uTrajectory[k]);
    problemMetrics.intermediates.push_back(
        computeIntermediateMetrics(problem, tTrajectory[k], xTrajectory[k], uTrajectory[k], dualSolution.intermediates[k]));

    // event time cost and constraints
    if (nextPostEventIndexItr != postEventIndices.end() && k + 1 == *nextPostEventIndexItr) {
      const auto m = dualSolution.preJumps[std::distance(postEventIndices.begin(), nextPostEventIndexItr)];
      problem.preComputationPtr->requestPreJump(request, tTrajectory[k], xTrajectory[k]);
      problemMetrics.preJumps.push_back(computePreJumpMetrics(problem, tTrajectory[k], xTrajectory[k], m));
      nextPostEventIndexItr++;
    }
  }

  // final time cost and constraints
  if (!tTrajectory.empty()) {
    problem.preComputationPtr->requestFinal(request, tTrajectory.back(), xTrajectory.back());
    problemMetrics.final = computeFinalMetrics(problem, tTrajectory.back(), xTrajectory.back(), dualSolution.final);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t& timeTrajectory, const ProblemMetrics& problemMetrics) {
  assert(timeTrajectory.size() == problemMetrics.intermediates.size());

  // Final
  PerformanceIndex performanceIndex = toPerformanceIndex(problemMetrics.final);

  // PreJumps
  std::for_each(problemMetrics.preJumps.cbegin(), problemMetrics.preJumps.cend(),
                [&performanceIndex](const Metrics& m) { performanceIndex += toPerformanceIndex(m); });

  // Intermediates
  std::vector<PerformanceIndex> performanceIndexTrajectory;
  performanceIndexTrajectory.reserve(problemMetrics.intermediates.size());
  std::for_each(problemMetrics.intermediates.cbegin(), problemMetrics.intermediates.cend(),
                [&performanceIndexTrajectory](const Metrics& m) { performanceIndexTrajectory.push_back(toPerformanceIndex(m)); });

  // Intermediates
  return trapezoidalIntegration(timeTrajectory, performanceIndexTrajectory, performanceIndex);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t rolloutTrajectory(RolloutBase& rollout, scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                           PrimalSolution& primalSolution) {
  // rollout with controller
  const auto xCurrent = rollout.run(initTime, initState, finalTime, primalSolution.controllerPtr_.get(), primalSolution.modeSchedule_,
                                    primalSolution.timeTrajectory_, primalSolution.postEventIndices_, primalSolution.stateTrajectory_,
                                    primalSolution.inputTrajectory_);

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("[rolloutTrajectory] System became unstable during the rollout!");
  }

  // average time step
  return (finalTime - initTime) / static_cast<scalar_t>(primalSolution.timeTrajectory_.size());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void projectLQ(const ModelData& modelData, const matrix_t& constraintRangeProjector, const matrix_t& constraintNullProjector,
               ModelData& projectedModelData) {
  // dimensions and time
  projectedModelData.time = modelData.time;
  projectedModelData.stateDim = modelData.stateDim;
  projectedModelData.inputDim = modelData.inputDim - modelData.stateInputEqConstraint.f.rows();

  // unhandled constraints
  projectedModelData.stateEqConstraint.f = vector_t();

  if (modelData.stateInputEqConstraint.f.rows() == 0) {
    // Change of variables u = Pu * tilde{u}
    // Pu = constraintNullProjector;

    // projected state-input equality constraints
    projectedModelData.stateInputEqConstraint.f.setZero(projectedModelData.inputDim);
    projectedModelData.stateInputEqConstraint.dfdx.setZero(projectedModelData.inputDim, projectedModelData.stateDim);
    projectedModelData.stateInputEqConstraint.dfdu.setZero(modelData.inputDim, modelData.inputDim);

    // dynamics
    projectedModelData.dynamics = modelData.dynamics;
    changeOfInputVariables(projectedModelData.dynamics, constraintNullProjector);

    // dynamics bias
    projectedModelData.dynamicsBias = modelData.dynamicsBias;

    // cost
    projectedModelData.cost = modelData.cost;
    changeOfInputVariables(projectedModelData.cost, constraintNullProjector);

  } else {
    // Change of variables u = Pu * tilde{u} + Px * x + u0
    // Pu = constraintNullProjector;
    // Px (= -CmProjected) = -constraintRangeProjector * C
    // u0 (= -EvProjected) = -constraintRangeProjector * e

    /* projected state-input equality constraints */
    projectedModelData.stateInputEqConstraint.f.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.f;
    projectedModelData.stateInputEqConstraint.dfdx.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.dfdx;
    projectedModelData.stateInputEqConstraint.dfdu.noalias() = constraintRangeProjector * modelData.stateInputEqConstraint.dfdu;

    // Change of variable matrices
    const auto& Pu = constraintNullProjector;
    const matrix_t Px = -projectedModelData.stateInputEqConstraint.dfdx;
    const matrix_t u0 = -projectedModelData.stateInputEqConstraint.f;

    // dynamics
    projectedModelData.dynamics = modelData.dynamics;
    changeOfInputVariables(projectedModelData.dynamics, Pu, Px, u0);

    // dynamics bias
    projectedModelData.dynamicsBias = modelData.dynamicsBias;
    projectedModelData.dynamicsBias.noalias() += modelData.dynamics.dfdu * u0;

    // cost
    projectedModelData.cost = modelData.cost;
    changeOfInputVariables(projectedModelData.cost, Pu, Px, u0);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void extractPrimalSolution(const std::pair<scalar_t, scalar_t>& timePeriod, const PrimalSolution& inputPrimalSolution,
                           PrimalSolution& outputPrimalSolution) {
  // no controller
  if (outputPrimalSolution.controllerPtr_ != nullptr) {
    outputPrimalSolution.controllerPtr_->clear();
  }
  // for none StateTriggeredRollout initialize modeSchedule
  outputPrimalSolution.modeSchedule_ = inputPrimalSolution.modeSchedule_;

  // create alias
  auto& timeTrajectory = outputPrimalSolution.timeTrajectory_;
  auto& stateTrajectory = outputPrimalSolution.stateTrajectory_;
  auto& inputTrajectory = outputPrimalSolution.inputTrajectory_;
  auto& postEventIndices = outputPrimalSolution.postEventIndices_;

  /*
   * Find the indexAlpha pair for interpolation. The interpolation function uses the std::lower_bound while ignoring the initial
   * time event, we should use std::upper_bound. Therefore at the first step, we check if for the case where std::upper_bound
   * would have give a different solution (index_alpha_t::second = 0) and correct the pair. Then, in the second step, we check
   * whether the index_alpha_t::first is a pre-event index. If yes, we move index_alpha_t::first to the post-event index.
   */
  const auto indexAlpha0 = [&]() {
    const auto lowerBoundIndexAlpha = LinearInterpolation::timeSegment(timePeriod.first, inputPrimalSolution.timeTrajectory_);

    const auto upperBoundIndexAlpha = numerics::almost_eq(lowerBoundIndexAlpha.second, 0.0)
                                          ? LinearInterpolation::index_alpha_t{lowerBoundIndexAlpha.first + 1, 1.0}
                                          : lowerBoundIndexAlpha;
    const auto it = std::find(inputPrimalSolution.postEventIndices_.cbegin(), inputPrimalSolution.postEventIndices_.cend(),
                              upperBoundIndexAlpha.first + 1);
    if (it == inputPrimalSolution.postEventIndices_.cend()) {
      return upperBoundIndexAlpha;
    } else {
      return LinearInterpolation::index_alpha_t{upperBoundIndexAlpha.first + 1, 1.0};
    }
  }();
  const auto indexAlpha1 = LinearInterpolation::timeSegment(timePeriod.second, inputPrimalSolution.timeTrajectory_);

  // time
  copySegment(indexAlpha0, indexAlpha1, inputPrimalSolution.timeTrajectory_, timeTrajectory);

  // state
  copySegment(indexAlpha0, indexAlpha1, inputPrimalSolution.stateTrajectory_, stateTrajectory);

  // input
  copySegment(indexAlpha0, indexAlpha1, inputPrimalSolution.inputTrajectory_, inputTrajectory);

  // If the pre-event index is within the range we accept the event
  postEventIndices.clear();
  for (const auto& postIndex : inputPrimalSolution.postEventIndices_) {
    if (postIndex > static_cast<size_t>(indexAlpha0.first) && inputPrimalSolution.timeTrajectory_[postIndex - 1] <= timePeriod.second) {
      postEventIndices.push_back(postIndex - static_cast<size_t>(indexAlpha0.first));
    }
  }

  // If there is an event at final time, it misses its pair (due to indexAlpha1 and copySegment)
  if (!postEventIndices.empty() && postEventIndices.back() == timeTrajectory.size()) {
    constexpr auto eps = numeric_traits::weakEpsilon<scalar_t>();
    const auto indexAlpha2 = LinearInterpolation::timeSegment(timePeriod.second + eps, inputPrimalSolution.timeTrajectory_);

    timeTrajectory.push_back(std::min(timePeriod.second + eps, timePeriod.second));
    stateTrajectory.push_back(LinearInterpolation::interpolate(indexAlpha2, inputPrimalSolution.stateTrajectory_));
    inputTrajectory.push_back(LinearInterpolation::interpolate(indexAlpha2, inputPrimalSolution.inputTrajectory_));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t maxControllerUpdateNorm(const LinearController& controller) {
  scalar_t maxDeltaUffNorm = 0.0;
  for (const auto& deltaBias : controller.deltaBiasArray_) {
    maxDeltaUffNorm = std::max(maxDeltaUffNorm, deltaBias.norm());
  }
  return maxDeltaUffNorm;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeControllerUpdateIS(const LinearController& controller) {
  scalar_array_t biasArraySquaredNorm(controller.timeStamp_.size());
  std::transform(controller.deltaBiasArray_.begin(), controller.deltaBiasArray_.end(), biasArraySquaredNorm.begin(),
                 [](const vector_t& b) { return b.squaredNorm(); });
  // integrates using the trapezoidal approximation method
  return trapezoidalIntegration(controller.timeStamp_, biasArraySquaredNorm, 0.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void incrementController(scalar_t stepLength, const LinearController& unoptimizedController, LinearController& controller) {
  controller.clear();
  controller.timeStamp_ = unoptimizedController.timeStamp_;
  controller.gainArray_ = unoptimizedController.gainArray_;
  controller.biasArray_.resize(unoptimizedController.size());
  for (size_t k = 0; k < unoptimizedController.size(); k++) {
    controller.biasArray_[k] = unoptimizedController.biasArray_[k] + stepLength * unoptimizedController.deltaBiasArray_[k];
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void retrieveActiveNormalizedTime(const std::pair<int, int>& partitionInterval, const scalar_array_t& timeTrajectory,
                                  const size_array_t& postEventIndices, scalar_array_t& normalizedTimeTrajectory,
                                  size_array_t& normalizedPostEventIndices) {
  // Although the rightmost point is excluded from the current interval, i.e. it won't be written into the dual solution array, we still
  // the following two (+1) are essential to start the backward pass
  auto firstTimeItr = timeTrajectory.begin() + partitionInterval.first;
  auto lastTimeItr = timeTrajectory.begin() + partitionInterval.second + 1;
  const int N = partitionInterval.second - partitionInterval.first + 1;
  // normalized time
  normalizedTimeTrajectory.resize(N);
  std::transform(firstTimeItr, lastTimeItr, normalizedTimeTrajectory.rbegin(), [](scalar_t t) -> scalar_t { return -t; });

  auto firstEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.first);
  auto lastEventItr = std::upper_bound(postEventIndices.begin(), postEventIndices.end(), partitionInterval.second);
  const int NE = std::distance(firstEventItr, lastEventItr);
  // normalized event past the index
  normalizedPostEventIndices.resize(NE);
  std::transform(firstEventItr, lastEventItr, normalizedPostEventIndices.rbegin(),
                 [N, &partitionInterval](size_t i) -> size_t { return N - i + partitionInterval.first; });
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<std::pair<int, int>> computePartitionIntervals(const scalar_array_t& timeTrajectory, int numWorkers) {
  const scalar_t increment = (timeTrajectory.back() - timeTrajectory.front()) / static_cast<scalar_t>(numWorkers);

  scalar_array_t desiredPartitionPoints(numWorkers + 1);
  desiredPartitionPoints.front() = timeTrajectory.front();
  for (size_t i = 1u; i < desiredPartitionPoints.size() - 1; i++) {
    desiredPartitionPoints[i] = desiredPartitionPoints[i - 1] + increment;
  }
  desiredPartitionPoints.back() = timeTrajectory.back();

  std::vector<std::pair<int, int>> partitionIntervals;
  partitionIntervals.reserve(desiredPartitionPoints.size());

  int endPos, startPos = 0;
  for (size_t i = 1u; i < desiredPartitionPoints.size(); i++) {
    const auto itr = std::upper_bound(timeTrajectory.begin(), timeTrajectory.end(), desiredPartitionPoints[i]);
    endPos = (itr != timeTrajectory.end()) ? std::distance(timeTrajectory.begin(), itr) : (timeTrajectory.size() - 1);
    if (endPos != startPos) {
      partitionIntervals.emplace_back(startPos, endPos);
      startPos = endPos;
    }
  }

  return partitionIntervals;
}

}  // namespace ocs2
