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

#include "ocs2_ddp/search_strategy/SearchStrategyBase.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SearchStrategyBase::initalize(scalar_t initTime, const vector_t& initState, scalar_t finalTime,
                                   const scalar_array_t& partitioningTimes, size_t initActivePartition, size_t finalActivePartition) {
  initTime_ = initTime;
  initState_ = initState;
  finalTime_ = finalTime;

  partitioningTimes_ = partitioningTimes;
  numPartitions_ = partitioningTimes_.size() - 1;
  initActivePartition_ = initActivePartition;
  finalActivePartition_ = finalActivePartition;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SearchStrategyBase::rolloutTrajectory(RolloutBase& rollout, const ModeSchedule& modeSchedule,
                                               std::vector<LinearController>& controllersStock, scalar_array2_t& timeTrajectoriesStock,
                                               size_array2_t& postEventIndicesStock, vector_array2_t& stateTrajectoriesStock,
                                               vector_array2_t& inputTrajectoriesStock,
                                               std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                               std::vector<std::vector<ModelData>>& modelDataEventTimesStock) const {
  if (controllersStock.size() != numPartitions_) {
    throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
  }

  // prepare outputs
  timeTrajectoriesStock.resize(numPartitions_);
  postEventIndicesStock.resize(numPartitions_);
  stateTrajectoriesStock.resize(numPartitions_);
  inputTrajectoriesStock.resize(numPartitions_);
  modelDataTrajectoriesStock.resize(numPartitions_);
  modelDataEventTimesStock.resize(numPartitions_);
  for (size_t i = 0; i < numPartitions_; i++) {
    timeTrajectoriesStock[i].clear();
    postEventIndicesStock[i].clear();
    stateTrajectoriesStock[i].clear();
    inputTrajectoriesStock[i].clear();
    modelDataTrajectoriesStock[i].clear();
    modelDataEventTimesStock[i].clear();
  }

  size_t numSteps = 0;
  vector_t xCurrent = initState_;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    // start and end of rollout segment
    const scalar_t t0 = (i == initActivePartition_) ? initTime_ : partitioningTimes_[i];
    const scalar_t tf = (i == finalActivePartition_) ? finalTime_ : partitioningTimes_[i + 1];

    // Rollout with controller
    xCurrent = rollout.run(t0, xCurrent, tf, &controllersStock[i], modeSchedule.eventTimes, timeTrajectoriesStock[i],
                           postEventIndicesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);

    // update model data trajectory
    modelDataTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      modelDataTrajectoriesStock[i][k].time_ = timeTrajectoriesStock[i][k];
      modelDataTrajectoriesStock[i][k].stateDim_ = stateTrajectoriesStock[i][k].size();
      modelDataTrajectoriesStock[i][k].inputDim_ = inputTrajectoriesStock[i][k].size();
      modelDataTrajectoriesStock[i][k].dynamicsBias_.setZero(stateTrajectoriesStock[i][k].size());
    }

    // update model data at event times
    modelDataEventTimesStock[i].resize(postEventIndicesStock[i].size());
    for (size_t ke = 0; ke < postEventIndicesStock[i].size(); ke++) {
      const auto index = postEventIndicesStock[i][ke] - 1;
      modelDataEventTimesStock[i][ke].time_ = timeTrajectoriesStock[i][index];
      modelDataEventTimesStock[i][ke].stateDim_ = stateTrajectoriesStock[i][index].size();
      modelDataEventTimesStock[i][ke].inputDim_ = inputTrajectoriesStock[i][index].size();
      modelDataEventTimesStock[i][ke].dynamicsBias_.setZero(stateTrajectoriesStock[i][index].size());
    }

    // total number of steps
    numSteps += timeTrajectoriesStock[i].size();
  }  // end of i loop

  // debug print
  if (baseSettings_.debugPrintRollout) {
    for (size_t i = 0; i < numPartitions_; i++) {
      std::cerr << "\n++++++++++++++++++++++++++++++\n";
      std::cerr << " Partition: " << i;
      std::cerr << "\n++++++++++++++++++++++++++++++\n";
      RolloutBase::display(timeTrajectoriesStock[i], postEventIndicesStock[i], stateTrajectoriesStock[i], &inputTrajectoriesStock[i]);
    }
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
void SearchStrategyBase::rolloutCostAndConstraints(OptimalControlProblem& problem, const scalar_array2_t& timeTrajectoriesStock,
                                                   const size_array2_t& postEventIndicesStock,
                                                   const vector_array2_t& stateTrajectoriesStock,
                                                   const vector_array2_t& inputTrajectoriesStock,
                                                   std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                                   std::vector<std::vector<ModelData>>& modelDataEventTimesStock,
                                                   scalar_t& heuristicsValue) const {
  auto& preComputation = *problem.preComputationPtr;
  const auto& desiredTrajectory = *problem.costDesiredTrajectories;

  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    auto eventsPastTheEndItr = postEventIndicesStock[i].begin();
    for (size_t k = 0; k < timeTrajectoriesStock[i].size(); k++) {
      const auto t = timeTrajectoriesStock[i][k];
      const auto& x = stateTrajectoriesStock[i][k];
      const auto& u = inputTrajectoriesStock[i][k];
      auto& modelData = modelDataTrajectoriesStock[i][k];

      preComputation.request(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x, u);

      // intermediate cost
      modelData.cost_.f = problem.costPtr->getValue(t, x, u, desiredTrajectory, preComputation);
      modelData.cost_.f += problem.stateCostPtr->getValue(t, x, desiredTrajectory, preComputation);
      modelData.cost_.f += problem.softConstraintPtr->getValue(t, x, u, desiredTrajectory, preComputation);
      modelData.cost_.f += problem.stateSoftConstraintPtr->getValue(t, x, desiredTrajectory, preComputation);

      // state equality constraint
      modelData.stateEqConstr_.f = problem.stateEqualityConstraintPtr->getValue(t, x, preComputation);

      // state-input equality constraint
      modelData.stateInputEqConstr_.f = problem.equalityConstraintPtr->getValue(t, x, u, preComputation);

      // inequality constraints
      modelData.ineqConstr_.f = problem.inequalityConstraintPtr->getValue(t, x, u, preComputation);

      // event time cost and constraints
      if (eventsPastTheEndItr != postEventIndicesStock[i].end() && k + 1 == *eventsPastTheEndItr) {
        const auto ke = std::distance(postEventIndicesStock[i].begin(), eventsPastTheEndItr);
        auto& modelDataEvent = modelDataEventTimesStock[i][ke];

        preComputation.requestPreJump(Request::Cost + Request::Constraint + Request::SoftConstraint, t, x);

        // pre-jump cost
        modelDataEvent.cost_.f = problem.preJumpCostPtr->getValue(t, x, desiredTrajectory, preComputation);
        modelDataEvent.cost_.f += problem.preJumpSoftConstraintPtr->getValue(t, x, desiredTrajectory, preComputation);

        // pre-jump constraint
        modelDataEvent.stateEqConstr_.f = problem.preJumpEqualityConstraintPtr->getValue(t, x, preComputation);

        eventsPastTheEndItr++;
      }
    }  // end of k loop
  }    // end of i loop

  // calculate the Heuristics function at the final time
  const auto t = timeTrajectoriesStock[finalActivePartition_].back();
  const auto& x = stateTrajectoriesStock[finalActivePartition_].back();
  preComputation.requestFinal(Request::Cost + Request::SoftConstraint, t, x);
  heuristicsValue = problem.finalCostPtr->getValue(t, x, desiredTrajectory, preComputation);
  heuristicsValue += problem.finalSoftConstraintPtr->getValue(t, x, desiredTrajectory, preComputation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PerformanceIndex SearchStrategyBase::calculateRolloutPerformanceIndex(const SoftConstraintPenalty& ineqConstrPenalty,
                                                                      const scalar_array2_t& timeTrajectoriesStock,
                                                                      const std::vector<std::vector<ModelData>>& modelDataTrajectoriesStock,
                                                                      const std::vector<std::vector<ModelData>>& modelDataEventTimesStock,
                                                                      scalar_t heuristicsValue) const {
  PerformanceIndex performanceIndex;
  for (size_t i = initActivePartition_; i <= finalActivePartition_; i++) {
    // total cost
    scalar_array_t costTrajectory(timeTrajectoriesStock[i].size());
    std::transform(modelDataTrajectoriesStock[i].begin(), modelDataTrajectoriesStock[i].end(), costTrajectory.begin(),
                   [](const ModelData& m) { return m.cost_.f; });
    performanceIndex.totalCost += trapezoidalIntegration(timeTrajectoriesStock[i], costTrajectory);

    // state equality constraint's ISE
    scalar_array_t stateEqualityNorm2Trajectory(timeTrajectoriesStock[i].size());
    std::transform(modelDataTrajectoriesStock[i].begin(), modelDataTrajectoriesStock[i].end(), stateEqualityNorm2Trajectory.begin(),
                   [](const ModelData& m) { return m.stateEqConstr_.f.squaredNorm(); });
    performanceIndex.stateEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], stateEqualityNorm2Trajectory);

    // state-input equality constraint's ISE
    scalar_array_t stateInputEqualityNorm2Trajectory(timeTrajectoriesStock[i].size());
    std::transform(modelDataTrajectoriesStock[i].begin(), modelDataTrajectoriesStock[i].end(), stateInputEqualityNorm2Trajectory.begin(),
                   [](const ModelData& m) { return m.stateInputEqConstr_.f.squaredNorm(); });
    performanceIndex.stateInputEqConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], stateInputEqualityNorm2Trajectory);

    // inequality constraints violation ISE
    scalar_array_t inequalityNorm2Trajectory(timeTrajectoriesStock[i].size());
    std::transform(modelDataTrajectoriesStock[i].begin(), modelDataTrajectoriesStock[i].end(), inequalityNorm2Trajectory.begin(),
                   [this](const ModelData& m) { return m.ineqConstr_.f.cwiseMin(0.0).squaredNorm(); });
    performanceIndex.inequalityConstraintISE += trapezoidalIntegration(timeTrajectoriesStock[i], inequalityNorm2Trajectory);

    // inequality constraints penalty
    scalar_array_t inequalityPenaltyTrajectory(timeTrajectoriesStock[i].size());
    std::transform(modelDataTrajectoriesStock[i].begin(), modelDataTrajectoriesStock[i].end(), inequalityPenaltyTrajectory.begin(),
                   [&](const ModelData& m) { return ineqConstrPenalty.getValue(m.ineqConstr_.f); });

    performanceIndex.inequalityConstraintPenalty += trapezoidalIntegration(timeTrajectoriesStock[i], inequalityPenaltyTrajectory);

    // final cost and constraints
    for (const auto& me : modelDataEventTimesStock[i]) {
      performanceIndex.totalCost += me.cost_.f;
      performanceIndex.stateEqFinalConstraintSSE += me.stateEqConstr_.f.squaredNorm();
    }
  }  // end of i loop

  // heuristic function
  performanceIndex.totalCost += heuristicsValue;

  return performanceIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SearchStrategyBase::calculateControllerUpdateIS(const std::vector<LinearController>& controllersStock) const {
  scalar_t controllerUpdateIS = 0.0;
  for (const auto& controller : controllersStock) {
    scalar_array_t biasArraySquaredNorm(controller.timeStamp_.size());
    std::transform(controller.deltaBiasArray_.begin(), controller.deltaBiasArray_.end(), biasArraySquaredNorm.begin(),
                   [this](const vector_t& b) { return b.squaredNorm(); });
    // integrates using the trapezoidal approximation method
    controllerUpdateIS += trapezoidalIntegration(controller.timeStamp_, biasArraySquaredNorm);
  }  // end of controller loop

  return controllerUpdateIS;
}

}  // namespace ocs2
