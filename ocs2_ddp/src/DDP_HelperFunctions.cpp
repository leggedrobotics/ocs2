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
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>
#include <ocs2_oc/oc_data/Metrics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void computeRolloutMetrics(OptimalControlProblem& problem, const PrimalSolution& primalSolution, Metrics& metrics) {
  const auto& tTrajectory = primalSolution.timeTrajectory_;
  const auto& xTrajectory = primalSolution.stateTrajectory_;
  const auto& uTrajectory = primalSolution.inputTrajectory_;
  const auto& postEventIndices = primalSolution.postEventIndices_;

  metrics.events.clear();
  metrics.events.reserve(postEventIndices.size());
  metrics.intermediates.clear();
  metrics.intermediates.reserve(tTrajectory.size());
  auto nextPostEventIndexItr = postEventIndices.begin();
  const auto request = Request::Cost + Request::Constraint + Request::SoftConstraint;
  for (size_t k = 0; k < tTrajectory.size(); k++) {
    // intermediate time cost and constraints
    problem.preComputationPtr->request(request, tTrajectory[k], xTrajectory[k], uTrajectory[k]);
    metrics.intermediates.push_back(computeIntermediateMetrics(problem, tTrajectory[k], xTrajectory[k], uTrajectory[k]));

    // event time cost and constraints
    if (nextPostEventIndexItr != postEventIndices.end() && k + 1 == *nextPostEventIndexItr) {
      problem.preComputationPtr->requestPreJump(request, tTrajectory[k], xTrajectory[k]);
      metrics.events.push_back(computeEventMetrics(problem, tTrajectory[k], xTrajectory[k]));
      nextPostEventIndexItr++;
    }
  }

  // final time cost and constraints
  problem.preComputationPtr->requestFinal(request, tTrajectory.back(), xTrajectory.back());
  metrics.final = computeFinalMetrics(problem, tTrajectory.back(), xTrajectory.back());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex computeRolloutPerformanceIndex(const scalar_array_t& timeTrajectory, const Metrics& metrics) {
  assert(timeTrajectory.size() == metrics.intermediates.size());

  PerformanceIndex performanceIndex;

  // total cost
  scalar_array_t costTrajectory(timeTrajectory.size());
  std::transform(metrics.intermediates.begin(), metrics.intermediates.end(), costTrajectory.begin(),
                 [](const IntermediateMetrics& m) { return m.cost; });
  performanceIndex.totalCost = trapezoidalIntegration(timeTrajectory, costTrajectory);

  std::for_each(metrics.events.begin(), metrics.events.end(), [&](const EventMetrics& m) { return performanceIndex.totalCost + m.cost; });

  performanceIndex.totalCost += metrics.final.cost;

  // state equality constraint's ISE
  performanceIndex.stateEqConstraintISE = 0.0;

  // state-input equality constraint's ISE
  scalar_array_t stateInputEqualityNorm2Trajectory(timeTrajectory.size());
  std::transform(metrics.intermediates.begin(), metrics.intermediates.end(), stateInputEqualityNorm2Trajectory.begin(),
                 [](const IntermediateMetrics& m) { return m.stateInputEqConstraint.squaredNorm(); });
  performanceIndex.stateInputEqConstraintISE = trapezoidalIntegration(timeTrajectory, stateInputEqualityNorm2Trajectory);

  // inequality constraints violation ISE
  performanceIndex.inequalityConstraintISE = 0.0;

  // inequality constraints penalty
  scalar_array_t inequalityPenaltyTrajectory(timeTrajectory.size());
  std::transform(metrics.intermediates.begin(), metrics.intermediates.end(), inequalityPenaltyTrajectory.begin(),
                 [&](const IntermediateMetrics& m) { return m.stateIneqPenalty + m.stateInputIneqPenalty; });
  performanceIndex.inequalityConstraintPenalty = trapezoidalIntegration(timeTrajectory, inequalityPenaltyTrajectory);

  return performanceIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t rolloutTrajectory(RolloutBase& rollout, const scalar_t initTime, const vector_t& initState, const scalar_t finalTime,
                           const ModeSchedule& modeSchedule, LinearController& controller, PrimalSolution& primalSolution) {
  assert(primalSolution.controllerPtr_.get() != &controller);
  primalSolution.clear();

  // fill mode schedule
  primalSolution.modeSchedule_ = modeSchedule;

  // rollout with controller
  const auto xCurrent = rollout.run(initTime, initState, finalTime, &controller, modeSchedule.eventTimes, primalSolution.timeTrajectory_,
                                    primalSolution.postEventIndices_, primalSolution.stateTrajectory_, primalSolution.inputTrajectory_);

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // average time step
  return (finalTime - initTime) / static_cast<scalar_t>(primalSolution.timeTrajectory_.size());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeModelData(const PrimalSolution& primalSolution, std::vector<ModelData>& modelDataTrajectory,
                         std::vector<ModelData>& modelDataEventTimes) {
  // update model data trajectory
  modelDataTrajectory.clear();
  modelDataTrajectory.resize(primalSolution.timeTrajectory_.size());
  for (size_t k = 0; k < modelDataTrajectory.size(); k++) {
    modelDataTrajectory[k].time_ = primalSolution.timeTrajectory_[k];
    modelDataTrajectory[k].stateDim_ = primalSolution.stateTrajectory_[k].size();
    modelDataTrajectory[k].inputDim_ = primalSolution.inputTrajectory_[k].size();
    modelDataTrajectory[k].dynamicsBias_.setZero(primalSolution.stateTrajectory_[k].size());
  }

  // update model data at event times
  modelDataEventTimes.clear();
  modelDataEventTimes.resize(primalSolution.postEventIndices_.size());
  for (size_t ke = 0; ke < modelDataEventTimes.size(); ke++) {
    const auto index = primalSolution.postEventIndices_[ke] - 1;
    modelDataEventTimes[ke].time_ = primalSolution.timeTrajectory_[index];
    modelDataEventTimes[ke].stateDim_ = primalSolution.stateTrajectory_[index].size();
    modelDataEventTimes[ke].inputDim_ = primalSolution.inputTrajectory_[index].size();
    modelDataEventTimes[ke].dynamicsBias_.setZero(primalSolution.stateTrajectory_[index].size());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t computeControllerUpdateIS(const LinearController& controller) {
  scalar_array_t biasArraySquaredNorm(controller.timeStamp_.size());
  std::transform(controller.deltaBiasArray_.begin(), controller.deltaBiasArray_.end(), biasArraySquaredNorm.begin(),
                 [](const vector_t& b) { return b.squaredNorm(); });
  // integrates using the trapezoidal approximation method
  return trapezoidalIntegration(controller.timeStamp_, biasArraySquaredNorm);
}

}  // namespace ocs2
