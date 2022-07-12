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

#include "ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, const DualSolution& cachedDualSolution,
                            DualSolution& dualSolution) {
  // find the time period that we can interpolate the cached dual solution
  const auto timePeriod = std::make_pair(primalSolution.timeTrajectory_.front(), primalSolution.timeTrajectory_.back());
  const auto interpolatableTimePeriod =
      findIntersectionToExtendableInterval(cachedDualSolution.timeTrajectory, primalSolution.modeSchedule_.eventTimes, timePeriod);
  const bool interpolateTillFinalTime = numerics::almost_eq(interpolatableTimePeriod.second, timePeriod.second);

  // clear and set time
  dualSolution.clear();
  dualSolution.timeTrajectory = primalSolution.timeTrajectory_;
  dualSolution.postEventIndices = primalSolution.postEventIndices_;

  // final
  if (interpolateTillFinalTime && !cachedDualSolution.final.empty()) {
    dualSolution.final = cachedDualSolution.final;
  } else {
    initializeFinalMultiplierCollection(ocp, primalSolution.timeTrajectory_.back(), dualSolution.final);
  }

  // pre-jumps
  dualSolution.preJumps.resize(primalSolution.postEventIndices_.size());
  if (!primalSolution.postEventIndices_.empty()) {
    const auto firstEventTime = primalSolution.timeTrajectory_[primalSolution.postEventIndices_[0] - 1];
    const auto cacheEventIndexBias =
        getNumberOfPrecedingEvents(cachedDualSolution.timeTrajectory, cachedDualSolution.postEventIndices, firstEventTime);

    for (size_t i = 0; i < primalSolution.postEventIndices_.size(); i++) {
      const auto cachedTimeIndex = cacheEventIndexBias + i;
      const auto& time = primalSolution.timeTrajectory_[i];
      auto& multiplierCollection = dualSolution.preJumps[i];
      if (cachedTimeIndex < cachedDualSolution.preJumps.size()) {
        multiplierCollection = cachedDualSolution.preJumps[cachedTimeIndex];
      } else {
        initializePreJumpMultiplierCollection(ocp, time, multiplierCollection);
      }
    }
  }

  // intermediates
  dualSolution.intermediates.resize(primalSolution.timeTrajectory_.size());
  for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    const auto& time = primalSolution.timeTrajectory_[i];
    auto& multiplierCollection = dualSolution.intermediates[i];
    if (interpolatableTimePeriod.first <= time && time <= interpolatableTimePeriod.second) {
      multiplierCollection = getIntermediateDualSolutionAtTime(cachedDualSolution, time);
    } else {
      initializeIntermediateMultiplierCollection(ocp, time, multiplierCollection);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection) {
  ocp.finalEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.finalInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multiplierCollection) {
  ocp.preJumpEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.preJumpInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time,
                                                MultiplierCollection& multiplierCollection) {
  ocp.stateEqualityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateEq);
  ocp.stateInequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateIneq);
  ocp.equalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateInputEq);
  ocp.inequalityLagrangianPtr->initializeLagrangian(time, multiplierCollection.stateInputIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateDualSolution(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution, ProblemMetrics& problemMetrics,
                        DualSolutionRef dualSolution) {
  // final
  if (!primalSolution.timeTrajectory_.empty()) {
    const auto& time = primalSolution.timeTrajectory_.back();
    const auto& state = primalSolution.stateTrajectory_.back();
    auto& metricsCollection = problemMetrics.final;
    auto& multiplierCollection = dualSolution.final;
    updateFinalMultiplierCollection(ocp, time, state, metricsCollection, multiplierCollection);
  }

  // preJump
  assert(dualSolution.preJumps.size() == primalSolution.postEventIndices_.size());
  assert(problemMetrics.preJumps.size() == primalSolution.postEventIndices_.size());
  for (size_t i = 0; i < primalSolution.postEventIndices_.size(); i++) {
    const auto timeIndex = primalSolution.postEventIndices_[i] - 1;
    const auto& time = primalSolution.timeTrajectory_[timeIndex];
    const auto& state = primalSolution.stateTrajectory_[timeIndex];
    auto& metricsCollection = problemMetrics.preJumps[i];
    auto& multiplierCollection = dualSolution.preJumps[i];
    updatePreJumpMultiplierCollection(ocp, time, state, metricsCollection, multiplierCollection);
  }

  // intermediates
  assert(dualSolution.intermediates.size() == primalSolution.timeTrajectory_.size());
  assert(problemMetrics.intermediates.size() == primalSolution.timeTrajectory_.size());
  for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    const auto& time = primalSolution.timeTrajectory_[i];
    const auto& state = primalSolution.stateTrajectory_[i];
    const auto& input = primalSolution.inputTrajectory_[i];
    auto& metricsCollection = problemMetrics.intermediates[i];
    auto& multiplierCollection = dualSolution.intermediates[i];
    updateIntermediateMultiplierCollection(ocp, time, state, input, metricsCollection, multiplierCollection);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                     MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.finalEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.finalInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state,
                                       MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.preJumpEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.preJumpInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            MetricsCollection& metricsCollection, MultiplierCollection& multiplierCollection) {
  ocp.stateEqualityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateEqLagrangian, multiplierCollection.stateEq);
  ocp.stateInequalityLagrangianPtr->updateLagrangian(time, state, metricsCollection.stateIneqLagrangian, multiplierCollection.stateIneq);
  ocp.equalityLagrangianPtr->updateLagrangian(time, state, input, metricsCollection.stateInputEqLagrangian,
                                              multiplierCollection.stateInputEq);
  ocp.inequalityLagrangianPtr->updateLagrangian(time, state, input, metricsCollection.stateInputIneqLagrangian,
                                                multiplierCollection.stateInputIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const LagrangianMetrics* extractFinalTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                                 const MetricsCollection& metricsColl) {
  size_t index;
  if (ocp.finalEqualityLagrangianPtr->getTermIndex(name, index)) {
    return &metricsColl.stateEqLagrangian[index];

  } else if (ocp.finalInequalityLagrangianPtr->getTermIndex(name, index)) {
    return &metricsColl.stateIneqLagrangian[index];

  } else {
    return nullptr;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractPreJumpTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                               const std::vector<MetricsCollection>& metricsCollArray,
                               std::vector<LagrangianMetricsConstRef>& metricsArray) {
  metricsArray.clear();

  size_t index;
  if (ocp.preJumpEqualityLagrangianPtr->getTermIndex(name, index)) {
    metricsArray.reserve(metricsCollArray.size());
    for (const auto& metricsColl : metricsCollArray) {
      metricsArray.push_back(metricsColl.stateEqLagrangian[index]);
    }
    return true;

  } else if (ocp.preJumpInequalityLagrangianPtr->getTermIndex(name, index)) {
    metricsArray.reserve(metricsCollArray.size());
    for (const auto& metricsColl : metricsCollArray) {
      metricsArray.push_back(metricsColl.stateIneqLagrangian[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractIntermediateTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                    const std::vector<MetricsCollection>& metricsCollTraj,
                                    std::vector<LagrangianMetricsConstRef>& metricsTrajectory) {
  metricsTrajectory.clear();

  size_t index;
  if (ocp.equalityLagrangianPtr->getTermIndex(name, index)) {
    metricsTrajectory.reserve(metricsCollTraj.size());
    for (const auto& metricsColl : metricsCollTraj) {
      metricsTrajectory.push_back(metricsColl.stateInputEqLagrangian[index]);
    }
    return true;

  } else if (ocp.stateEqualityLagrangianPtr->getTermIndex(name, index)) {
    metricsTrajectory.reserve(metricsCollTraj.size());
    for (const auto& metricsColl : metricsCollTraj) {
      metricsTrajectory.push_back(metricsColl.stateEqLagrangian[index]);
    }
    return true;

  } else if (ocp.inequalityLagrangianPtr->getTermIndex(name, index)) {
    metricsTrajectory.reserve(metricsCollTraj.size());
    for (const auto& metricsColl : metricsCollTraj) {
      metricsTrajectory.push_back(metricsColl.stateInputIneqLagrangian[index]);
    }
    return true;

  } else if (ocp.stateInequalityLagrangianPtr->getTermIndex(name, index)) {
    metricsTrajectory.reserve(metricsCollTraj.size());
    for (const auto& metricsColl : metricsCollTraj) {
      metricsTrajectory.push_back(metricsColl.stateIneqLagrangian[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const Multiplier* extractFinalTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                             const MultiplierCollection& multiplierColl) {
  size_t index;
  if (ocp.finalEqualityLagrangianPtr->getTermIndex(name, index)) {
    return &multiplierColl.stateEq[index];

  } else if (ocp.finalInequalityLagrangianPtr->getTermIndex(name, index)) {
    return &multiplierColl.stateIneq[index];

  } else {
    return nullptr;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractPreJumpTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                  const std::vector<MultiplierCollection>& multiplierCollArray,
                                  std::vector<MultiplierConstRef>& multiplierArray) {
  multiplierArray.clear();

  size_t index;
  if (ocp.preJumpEqualityLagrangianPtr->getTermIndex(name, index)) {
    multiplierArray.reserve(multiplierCollArray.size());
    for (const auto& multiplierColl : multiplierCollArray) {
      multiplierArray.push_back(multiplierColl.stateEq[index]);
    }
    return true;

  } else if (ocp.preJumpInequalityLagrangianPtr->getTermIndex(name, index)) {
    multiplierArray.reserve(multiplierCollArray.size());
    for (const auto& multiplierColl : multiplierCollArray) {
      multiplierArray.push_back(multiplierColl.stateIneq[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractIntermediateTermMultiplier(const OptimalControlProblem& ocp, const std::string& name,
                                       const std::vector<MultiplierCollection>& multiplierCollTraj,
                                       std::vector<MultiplierConstRef>& multiplierTrajectory) {
  multiplierTrajectory.clear();

  size_t index;
  if (ocp.equalityLagrangianPtr->getTermIndex(name, index)) {
    multiplierTrajectory.reserve(multiplierCollTraj.size());
    for (const auto& multiplierColl : multiplierCollTraj) {
      multiplierTrajectory.push_back(multiplierColl.stateInputEq[index]);
    }
    return true;

  } else if (ocp.stateEqualityLagrangianPtr->getTermIndex(name, index)) {
    multiplierTrajectory.reserve(multiplierCollTraj.size());
    for (const auto& multiplierColl : multiplierCollTraj) {
      multiplierTrajectory.push_back(multiplierColl.stateEq[index]);
    }
    return true;

  } else if (ocp.inequalityLagrangianPtr->getTermIndex(name, index)) {
    multiplierTrajectory.reserve(multiplierCollTraj.size());
    for (const auto& multiplierColl : multiplierCollTraj) {
      multiplierTrajectory.push_back(multiplierColl.stateInputIneq[index]);
    }
    return true;

  } else if (ocp.stateInequalityLagrangianPtr->getTermIndex(name, index)) {
    multiplierTrajectory.reserve(multiplierCollTraj.size());
    for (const auto& multiplierColl : multiplierCollTraj) {
      multiplierTrajectory.push_back(multiplierColl.stateIneq[index]);
    }
    return true;

  } else {
    return false;
  }
}

}  // namespace ocs2
