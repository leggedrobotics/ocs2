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
      auto& multipliers = dualSolution.preJumps[i];
      if (cachedTimeIndex < cachedDualSolution.preJumps.size()) {
        multipliers = cachedDualSolution.preJumps[cachedTimeIndex];
      } else {
        initializePreJumpMultiplierCollection(ocp, time, multipliers);
      }
    }
  }

  // intermediates
  dualSolution.intermediates.resize(primalSolution.timeTrajectory_.size());
  for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    const auto& time = primalSolution.timeTrajectory_[i];
    auto& multipliers = dualSolution.intermediates[i];
    if (interpolatableTimePeriod.first <= time && time <= interpolatableTimePeriod.second) {
      multipliers = getIntermediateDualSolutionAtTime(cachedDualSolution, time);
    } else {
      initializeIntermediateMultiplierCollection(ocp, time, multipliers);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multipliers) {
  ocp.finalEqualityLagrangianPtr->initializeLagrangian(time, multipliers.stateEq);
  ocp.finalInequalityLagrangianPtr->initializeLagrangian(time, multipliers.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multipliers) {
  ocp.preJumpEqualityLagrangianPtr->initializeLagrangian(time, multipliers.stateEq);
  ocp.preJumpInequalityLagrangianPtr->initializeLagrangian(time, multipliers.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void initializeIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, MultiplierCollection& multipliers) {
  ocp.stateEqualityLagrangianPtr->initializeLagrangian(time, multipliers.stateEq);
  ocp.stateInequalityLagrangianPtr->initializeLagrangian(time, multipliers.stateIneq);
  ocp.equalityLagrangianPtr->initializeLagrangian(time, multipliers.stateInputEq);
  ocp.inequalityLagrangianPtr->initializeLagrangian(time, multipliers.stateInputIneq);
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
    auto& metrics = problemMetrics.final;
    auto& multipliers = dualSolution.final;
    updateFinalMultiplierCollection(ocp, time, state, metrics, multipliers);
  }

  // preJump
  assert(dualSolution.preJumps.size() == primalSolution.postEventIndices_.size());
  assert(problemMetrics.preJumps.size() == primalSolution.postEventIndices_.size());
  for (size_t i = 0; i < primalSolution.postEventIndices_.size(); i++) {
    const auto timeIndex = primalSolution.postEventIndices_[i] - 1;
    const auto& time = primalSolution.timeTrajectory_[timeIndex];
    const auto& state = primalSolution.stateTrajectory_[timeIndex];
    auto& metrics = problemMetrics.preJumps[i];
    auto& multipliers = dualSolution.preJumps[i];
    updatePreJumpMultiplierCollection(ocp, time, state, metrics, multipliers);
  }

  // intermediates
  assert(dualSolution.intermediates.size() == primalSolution.timeTrajectory_.size());
  assert(problemMetrics.intermediates.size() == primalSolution.timeTrajectory_.size());
  for (size_t i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    const auto& time = primalSolution.timeTrajectory_[i];
    const auto& state = primalSolution.stateTrajectory_[i];
    const auto& input = primalSolution.inputTrajectory_[i];
    auto& metrics = problemMetrics.intermediates[i];
    auto& multipliers = dualSolution.intermediates[i];
    updateIntermediateMultiplierCollection(ocp, time, state, input, metrics, multipliers);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateFinalMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                     MultiplierCollection& multipliers) {
  ocp.finalEqualityLagrangianPtr->updateLagrangian(time, state, metrics.stateEqLagrangian, multipliers.stateEq);
  ocp.finalInequalityLagrangianPtr->updateLagrangian(time, state, metrics.stateIneqLagrangian, multipliers.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updatePreJumpMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, Metrics& metrics,
                                       MultiplierCollection& multipliers) {
  ocp.preJumpEqualityLagrangianPtr->updateLagrangian(time, state, metrics.stateEqLagrangian, multipliers.stateEq);
  ocp.preJumpInequalityLagrangianPtr->updateLagrangian(time, state, metrics.stateIneqLagrangian, multipliers.stateIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void updateIntermediateMultiplierCollection(const OptimalControlProblem& ocp, scalar_t time, const vector_t& state, const vector_t& input,
                                            Metrics& metrics, MultiplierCollection& multipliers) {
  ocp.stateEqualityLagrangianPtr->updateLagrangian(time, state, metrics.stateEqLagrangian, multipliers.stateEq);
  ocp.stateInequalityLagrangianPtr->updateLagrangian(time, state, metrics.stateIneqLagrangian, multipliers.stateIneq);
  ocp.equalityLagrangianPtr->updateLagrangian(time, state, input, metrics.stateInputEqLagrangian, multipliers.stateInputEq);
  ocp.inequalityLagrangianPtr->updateLagrangian(time, state, input, metrics.stateInputIneqLagrangian, multipliers.stateInputIneq);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const vector_t* extractFinalTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const Metrics& metrics) {
  size_t index;
  if (ocp.finalEqualityConstraintPtr->getTermIndex(name, index)) {
    return &metrics.stateEqConstraint[index];

  } else if (ocp.finalInequalityConstraintPtr->getTermIndex(name, index)) {
    return &metrics.stateIneqConstraint[index];

  } else {
    return nullptr;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const LagrangianMetrics* extractFinalTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                                           const Metrics& metrics) {
  size_t index;
  if (ocp.finalEqualityLagrangianPtr->getTermIndex(name, index)) {
    return &metrics.stateEqLagrangian[index];

  } else if (ocp.finalInequalityLagrangianPtr->getTermIndex(name, index)) {
    return &metrics.stateIneqLagrangian[index];

  } else {
    return nullptr;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractPreJumpTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsArray,
                                  std::vector<std::reference_wrapper<const vector_t>>& constraintArray) {
  constraintArray.clear();

  size_t index;
  if (ocp.preJumpEqualityConstraintPtr->getTermIndex(name, index)) {
    constraintArray.reserve(metricsArray.size());
    for (const auto& m : metricsArray) {
      constraintArray.emplace_back(m.stateEqConstraint[index]);
    }
    return true;

  } else if (ocp.preJumpInequalityConstraintPtr->getTermIndex(name, index)) {
    constraintArray.reserve(metricsArray.size());
    for (const auto& m : metricsArray) {
      constraintArray.emplace_back(m.stateIneqConstraint[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractPreJumpTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                         const std::vector<Metrics>& metricsArray,
                                         std::vector<LagrangianMetricsConstRef>& lagrangianMetricsArray) {
  lagrangianMetricsArray.clear();

  size_t index;
  if (ocp.preJumpEqualityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsArray.reserve(metricsArray.size());
    for (const auto& m : metricsArray) {
      lagrangianMetricsArray.push_back(m.stateEqLagrangian[index]);
    }
    return true;

  } else if (ocp.preJumpInequalityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsArray.reserve(metricsArray.size());
    for (const auto& m : metricsArray) {
      lagrangianMetricsArray.push_back(m.stateIneqLagrangian[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractIntermediateTermConstraint(const OptimalControlProblem& ocp, const std::string& name, const std::vector<Metrics>& metricsTraj,
                                       std::vector<std::reference_wrapper<const vector_t>>& constraintTraj) {
  constraintTraj.clear();

  size_t index;
  if (ocp.equalityConstraintPtr->getTermIndex(name, index)) {
    constraintTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      constraintTraj.emplace_back(m.stateInputEqConstraint[index]);
    }
    return true;

  } else if (ocp.stateEqualityConstraintPtr->getTermIndex(name, index)) {
    constraintTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      constraintTraj.emplace_back(m.stateEqConstraint[index]);
    }
    return true;

  } else if (ocp.inequalityConstraintPtr->getTermIndex(name, index)) {
    constraintTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      constraintTraj.emplace_back(m.stateInputIneqConstraint[index]);
    }
    return true;

  } else if (ocp.stateInequalityConstraintPtr->getTermIndex(name, index)) {
    constraintTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      constraintTraj.emplace_back(m.stateIneqConstraint[index]);
    }
    return true;

  } else {
    return false;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool extractIntermediateTermLagrangianMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                              const std::vector<Metrics>& metricsTraj,
                                              std::vector<LagrangianMetricsConstRef>& lagrangianMetricsTraj) {
  lagrangianMetricsTraj.clear();

  size_t index;
  if (ocp.equalityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      lagrangianMetricsTraj.push_back(m.stateInputEqLagrangian[index]);
    }
    return true;

  } else if (ocp.stateEqualityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      lagrangianMetricsTraj.push_back(m.stateEqLagrangian[index]);
    }
    return true;

  } else if (ocp.inequalityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      lagrangianMetricsTraj.push_back(m.stateInputIneqLagrangian[index]);
    }
    return true;

  } else if (ocp.stateInequalityLagrangianPtr->getTermIndex(name, index)) {
    lagrangianMetricsTraj.reserve(metricsTraj.size());
    for (const auto& m : metricsTraj) {
      lagrangianMetricsTraj.push_back(m.stateIneqLagrangian[index]);
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
