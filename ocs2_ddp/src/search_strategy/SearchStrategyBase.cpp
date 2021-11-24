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
void SearchStrategyBase::initalize(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  initTime_ = initTime;
  initState_ = initState;
  finalTime_ = finalTime;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SearchStrategyBase::rolloutTrajectory(RolloutBase& rollout, const ModeSchedule& modeSchedule, LinearController& controllersStock,
                                               scalar_array_t& timeTrajectoriesStock, size_array_t& postEventIndicesStock,
                                               vector_array_t& stateTrajectoriesStock, vector_array_t& inputTrajectoriesStock,
                                               std::vector<ModelData>& modelDataTrajectoriesStock,
                                               std::vector<ModelData>& modelDataEventTimesStock) const {
  // prepare outputs
  timeTrajectoriesStock.clear();
  postEventIndicesStock.clear();
  stateTrajectoriesStock.clear();
  inputTrajectoriesStock.clear();
  modelDataTrajectoriesStock.clear();
  modelDataEventTimesStock.clear();

  size_t numSteps = 0;
  vector_t xCurrent = initState_;
  // start and end of rollout segment
  const scalar_t t0 = initTime_;
  const scalar_t tf = finalTime_;

  // Rollout with controller
  xCurrent = rollout.run(t0, xCurrent, tf, &controllersStock, modeSchedule.eventTimes, timeTrajectoriesStock, postEventIndicesStock,
                         stateTrajectoriesStock, inputTrajectoriesStock);

  // update model data trajectory
  modelDataTrajectoriesStock.resize(timeTrajectoriesStock.size());
  for (size_t k = 0; k < timeTrajectoriesStock.size(); k++) {
    modelDataTrajectoriesStock[k].time_ = timeTrajectoriesStock[k];
    modelDataTrajectoriesStock[k].stateDim_ = stateTrajectoriesStock[k].size();
    modelDataTrajectoriesStock[k].inputDim_ = inputTrajectoriesStock[k].size();
    modelDataTrajectoriesStock[k].dynamicsBias_.setZero(stateTrajectoriesStock[k].size());
  }

  // update model data at event times
  modelDataEventTimesStock.resize(postEventIndicesStock.size());
  for (size_t ke = 0; ke < postEventIndicesStock.size(); ke++) {
    const auto index = postEventIndicesStock[ke] - 1;
    modelDataEventTimesStock[ke].time_ = timeTrajectoriesStock[index];
    modelDataEventTimesStock[ke].stateDim_ = stateTrajectoriesStock[index].size();
    modelDataEventTimesStock[ke].inputDim_ = inputTrajectoriesStock[index].size();
    modelDataEventTimesStock[ke].dynamicsBias_.setZero(stateTrajectoriesStock[index].size());
  }

  // total number of steps
  numSteps += timeTrajectoriesStock.size();

  // debug print
  if (baseSettings_.debugPrintRollout) {
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    std::cerr << "\n++++++++++++++++++++++++++++++\n";
    RolloutBase::display(timeTrajectoriesStock, postEventIndicesStock, stateTrajectoriesStock, &inputTrajectoriesStock);
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
void SearchStrategyBase::rolloutCostAndConstraints(OptimalControlProblem& problem, const scalar_array_t& timeTrajectoriesStock,
                                                   const size_array_t& postEventIndicesStock, const vector_array_t& stateTrajectoriesStock,
                                                   const vector_array_t& inputTrajectoriesStock,
                                                   std::vector<ModelData>& modelDataTrajectoriesStock,
                                                   std::vector<ModelData>& modelDataEventTimesStock, scalar_t& heuristicsValue) const {
  auto& preComputation = *problem.preComputationPtr;

  auto eventsPastTheEndItr = postEventIndicesStock.begin();
  for (size_t k = 0; k < timeTrajectoriesStock.size(); k++) {
    const auto t = timeTrajectoriesStock[k];
    const auto& x = stateTrajectoriesStock[k];
    const auto& u = inputTrajectoriesStock[k];
    auto& modelData = modelDataTrajectoriesStock[k];

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
    if (eventsPastTheEndItr != postEventIndicesStock.end() && k + 1 == *eventsPastTheEndItr) {
      const auto ke = std::distance(postEventIndicesStock.begin(), eventsPastTheEndItr);
      auto& modelDataEvent = modelDataEventTimesStock[ke];

      preComputation.requestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x);

      // pre-jump cost
      modelDataEvent.cost_.f = computeEventCost(problem, t, x);

      // pre-jump constraint
      modelDataEvent.stateEqConstr_.f = problem.preJumpEqualityConstraintPtr->getValue(t, x, preComputation);

      eventsPastTheEndItr++;
    }
  }  // end of k loop

  // calculate the Heuristics function at the final time
  const auto t = timeTrajectoriesStock.back();
  const auto& x = stateTrajectoriesStock.back();
  preComputation.requestFinal(Request::Cost + Request::SoftConstraint, t, x);
  heuristicsValue = computeFinalCost(problem, t, x);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex SearchStrategyBase::calculateRolloutPerformanceIndex(const SoftConstraintPenalty& ineqConstrPenalty,
                                                                      const scalar_array_t& timeTrajectoriesStock,
                                                                      const std::vector<ModelData>& modelDataTrajectoriesStock,
                                                                      const std::vector<ModelData>& modelDataEventTimesStock,
                                                                      scalar_t heuristicsValue) const {
  PerformanceIndex performanceIndex;
  // total cost
  scalar_array_t costTrajectory(timeTrajectoriesStock.size());
  std::transform(modelDataTrajectoriesStock.begin(), modelDataTrajectoriesStock.end(), costTrajectory.begin(),
                 [](const ModelData& m) { return m.cost_.f; });
  performanceIndex.totalCost += trapezoidalIntegration(timeTrajectoriesStock, costTrajectory);

  // state equality constraint's ISE
  scalar_array_t stateEqualityNorm2Trajectory(timeTrajectoriesStock.size());
  std::transform(modelDataTrajectoriesStock.begin(), modelDataTrajectoriesStock.end(), stateEqualityNorm2Trajectory.begin(),
                 [](const ModelData& m) { return m.stateEqConstr_.f.squaredNorm(); });
  performanceIndex.stateEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock, stateEqualityNorm2Trajectory);

  // state-input equality constraint's ISE
  scalar_array_t stateInputEqualityNorm2Trajectory(timeTrajectoriesStock.size());
  std::transform(modelDataTrajectoriesStock.begin(), modelDataTrajectoriesStock.end(), stateInputEqualityNorm2Trajectory.begin(),
                 [](const ModelData& m) { return m.stateInputEqConstr_.f.squaredNorm(); });
  performanceIndex.stateInputEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock, stateInputEqualityNorm2Trajectory);

  // inequality constraints violation ISE
  scalar_array_t inequalityNorm2Trajectory(timeTrajectoriesStock.size());
  std::transform(modelDataTrajectoriesStock.begin(), modelDataTrajectoriesStock.end(), inequalityNorm2Trajectory.begin(),
                 [this](const ModelData& m) { return m.ineqConstr_.f.cwiseMin(0.0).squaredNorm(); });
  performanceIndex.inequalityConstraintISE += trapezoidalIntegration(timeTrajectoriesStock, inequalityNorm2Trajectory);

  // inequality constraints penalty
  scalar_array_t inequalityPenaltyTrajectory(timeTrajectoriesStock.size());
  std::transform(modelDataTrajectoriesStock.begin(), modelDataTrajectoriesStock.end(), inequalityPenaltyTrajectory.begin(),
                 [&](const ModelData& m) { return ineqConstrPenalty.getValue(m.time_, m.ineqConstr_.f); });

  performanceIndex.inequalityConstraintPenalty += trapezoidalIntegration(timeTrajectoriesStock, inequalityPenaltyTrajectory);

  // final cost and constraints
  for (const auto& me : modelDataEventTimesStock) {
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
