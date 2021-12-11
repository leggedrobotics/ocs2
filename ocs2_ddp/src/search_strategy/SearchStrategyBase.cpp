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

#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

#include "ocs2_ddp/search_strategy/SearchStrategyBase.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SearchStrategyBase::rolloutTrajectory(RolloutBase& rollout, const ModeSchedule& modeSchedule, LinearController& controller,
                                               PrimalDataContainer& dstPrimalData) const {
  assert(dstPrimalData.primalSolution.controllerPtr_.get() != &controller);
  // prepare outputs
  dstPrimalData.clear();
  // fill mode schedule
  dstPrimalData.primalSolution.modeSchedule_ = modeSchedule;
  // create alias
  auto& timeTrajectory = dstPrimalData.primalSolution.timeTrajectory_;
  auto& stateTrajectory = dstPrimalData.primalSolution.stateTrajectory_;
  auto& inputTrajectory = dstPrimalData.primalSolution.inputTrajectory_;
  auto& postEventIndices = dstPrimalData.postEventIndices;
  auto& modelDataTrajectory = dstPrimalData.modelDataTrajectory;
  auto& modelDataEventTimes = dstPrimalData.modelDataEventTimes;

  // Rollout with controller
  vector_t xCurrent = rollout.run(initTime_, initState_, finalTime_, &controller, modeSchedule.eventTimes, timeTrajectory, postEventIndices,
                                  stateTrajectory, inputTrajectory);

  // update model data trajectory
  modelDataTrajectory.resize(timeTrajectory.size());
  for (size_t k = 0; k < timeTrajectory.size(); k++) {
    modelDataTrajectory[k].time_ = timeTrajectory[k];
    modelDataTrajectory[k].stateDim_ = stateTrajectory[k].size();
    modelDataTrajectory[k].inputDim_ = inputTrajectory[k].size();
    modelDataTrajectory[k].dynamicsBias_.setZero(stateTrajectory[k].size());
  }

  // update model data at event times
  modelDataEventTimes.resize(postEventIndices.size());
  for (size_t ke = 0; ke < postEventIndices.size(); ke++) {
    const auto index = postEventIndices[ke] - 1;
    modelDataEventTimes[ke].time_ = timeTrajectory[index];
    modelDataEventTimes[ke].stateDim_ = stateTrajectory[index].size();
    modelDataEventTimes[ke].inputDim_ = inputTrajectory[index].size();
    modelDataEventTimes[ke].dynamicsBias_.setZero(stateTrajectory[index].size());
  }

  // total number of steps
  size_t numSteps = timeTrajectory.size();

  // debug print
  if (baseSettings_.debugPrintRollout) {
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    RolloutBase::display(timeTrajectory, postEventIndices, stateTrajectory, &inputTrajectory);
  }

  if (!xCurrent.allFinite()) {
    throw std::runtime_error("System became unstable during the rollout.");
  }

  // average time step
  return (finalTime_ - initTime_) / numSteps;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SearchStrategyBase::rolloutCostAndConstraints(OptimalControlProblem& problem, PrimalDataContainer& primalData,
                                                   scalar_t& heuristicsValue) const {
  // create alias
  auto& preComputation = *problem.preComputationPtr;

  const auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  const auto& postEventIndices = primalData.postEventIndices;
  const auto& stateTrajectory = primalData.primalSolution.stateTrajectory_;
  const auto& inputTrajectory = primalData.primalSolution.inputTrajectory_;
  auto& modelDataTrajectory = primalData.modelDataTrajectory;
  auto& modelDataEventTimes = primalData.modelDataEventTimes;

  auto nextPostEventIndexItr = postEventIndices.begin();
  for (size_t k = 0; k < timeTrajectory.size(); k++) {
    const auto t = timeTrajectory[k];
    const auto& x = stateTrajectory[k];
    const auto& u = inputTrajectory[k];
    auto& modelData = modelDataTrajectory[k];

    preComputation.request(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x, u);

    // intermediate cost
    modelData.cost_.f = computeCost(problem, t, x, u);

    // state equality constraint
    modelData.stateEqConstr_.f = problem.stateEqualityConstraintPtr->getValue(t, x, preComputation);

    // state-input equality constraint
    modelData.stateInputEqConstr_.f = problem.equalityConstraintPtr->getValue(t, x, u, preComputation);

    // inequality constraints
    modelData.ineqConstr_.f = problem.inequalityConstraintPtr->getValue(t, x, u, preComputation);

    // event time cost and constraints
    if (nextPostEventIndexItr != postEventIndices.end() && k + 1 == *nextPostEventIndexItr) {
      const auto ke = std::distance(postEventIndices.begin(), nextPostEventIndexItr);
      auto& modelDataEvent = modelDataEventTimes[ke];

      preComputation.requestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x);

      // pre-jump cost
      modelDataEvent.cost_.f = computeEventCost(problem, t, x);

      // pre-jump constraint
      modelDataEvent.stateEqConstr_.f = problem.preJumpEqualityConstraintPtr->getValue(t, x, preComputation);

      nextPostEventIndexItr++;
    }
  }  // end of k loop

  // calculate the Heuristics function at the final time
  preComputation.requestFinal(Request::Cost + Request::SoftConstraint, timeTrajectory.back(), stateTrajectory.back());
  heuristicsValue = computeFinalCost(problem, timeTrajectory.back(), stateTrajectory.back());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex SearchStrategyBase::calculateRolloutPerformanceIndex(const SoftConstraintPenalty& ineqConstrPenalty,
                                                                      const PrimalDataContainer& primalData,
                                                                      scalar_t heuristicsValue) const {
  // create alias
  const auto& timeTrajectory = primalData.primalSolution.timeTrajectory_;
  const auto& modelDataTrajectory = primalData.modelDataTrajectory;
  const auto& modelDataEventTimes = primalData.modelDataEventTimes;

  PerformanceIndex performanceIndex;
  // total cost
  scalar_array_t costTrajectory(timeTrajectory.size());
  std::transform(modelDataTrajectory.begin(), modelDataTrajectory.end(), costTrajectory.begin(),
                 [](const ModelData& m) { return m.cost_.f; });
  performanceIndex.totalCost += trapezoidalIntegration(timeTrajectory, costTrajectory);

  // state equality constraint's ISE
  scalar_array_t stateEqualityNorm2Trajectory(timeTrajectory.size());
  std::transform(modelDataTrajectory.begin(), modelDataTrajectory.end(), stateEqualityNorm2Trajectory.begin(),
                 [](const ModelData& m) { return m.stateEqConstr_.f.squaredNorm(); });
  performanceIndex.stateEqConstraintISE += trapezoidalIntegration(timeTrajectory, stateEqualityNorm2Trajectory);

  // state-input equality constraint's ISE
  scalar_array_t stateInputEqualityNorm2Trajectory(timeTrajectory.size());
  std::transform(modelDataTrajectory.begin(), modelDataTrajectory.end(), stateInputEqualityNorm2Trajectory.begin(),
                 [](const ModelData& m) { return m.stateInputEqConstr_.f.squaredNorm(); });
  performanceIndex.stateInputEqConstraintISE += trapezoidalIntegration(timeTrajectory, stateInputEqualityNorm2Trajectory);

  // inequality constraints violation ISE
  scalar_array_t inequalityNorm2Trajectory(timeTrajectory.size());
  std::transform(modelDataTrajectory.begin(), modelDataTrajectory.end(), inequalityNorm2Trajectory.begin(),
                 [this](const ModelData& m) { return m.ineqConstr_.f.cwiseMin(0.0).squaredNorm(); });
  performanceIndex.inequalityConstraintISE += trapezoidalIntegration(timeTrajectory, inequalityNorm2Trajectory);

  // inequality constraints penalty
  scalar_array_t inequalityPenaltyTrajectory(timeTrajectory.size());
  std::transform(modelDataTrajectory.begin(), modelDataTrajectory.end(), inequalityPenaltyTrajectory.begin(),
                 [&](const ModelData& m) { return ineqConstrPenalty.getValue(m.time_, m.ineqConstr_.f); });

  performanceIndex.inequalityConstraintPenalty += trapezoidalIntegration(timeTrajectory, inequalityPenaltyTrajectory);

  // final cost and constraints
  for (const auto& me : modelDataEventTimes) {
    performanceIndex.totalCost += me.cost_.f;
    performanceIndex.stateEqFinalConstraintSSE += me.stateEqConstr_.f.squaredNorm();
  }

  // heuristic function
  performanceIndex.totalCost += heuristicsValue;

  return performanceIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SearchStrategyBase::calculateControllerUpdateIS(const LinearController& controller) const {
  scalar_array_t biasArraySquaredNorm(controller.timeStamp_.size());
  std::transform(controller.deltaBiasArray_.begin(), controller.deltaBiasArray_.end(), biasArraySquaredNorm.begin(),
                 [](const vector_t& b) { return b.squaredNorm(); });
  // integrates using the trapezoidal approximation method
  return trapezoidalIntegration(controller.timeStamp_, biasArraySquaredNorm);
}

}  // namespace ocs2
