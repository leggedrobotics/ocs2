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

#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem()
    : /* Cost */
      costPtr(new StateInputCostCollection),
      stateCostPtr(new StateCostCollection),
      preJumpCostPtr(new StateCostCollection),
      finalCostPtr(new StateCostCollection),
      /* Soft constraints */
      softConstraintPtr(new StateInputCostCollection),
      stateSoftConstraintPtr(new StateCostCollection),
      preJumpSoftConstraintPtr(new StateCostCollection),
      finalSoftConstraintPtr(new StateCostCollection),
      /* Equality constraints */
      equalityConstraintPtr(new StateInputConstraintCollection),
      stateEqualityConstraintPtr(new StateConstraintCollection),
      preJumpEqualityConstraintPtr(new StateConstraintCollection),
      finalEqualityConstraintPtr(new StateConstraintCollection),
      /* Lagrangians */
      equalityLagrangianPtr(new StateInputAugmentedLagrangianCollection),
      stateEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      inequalityLagrangianPtr(new StateInputAugmentedLagrangianCollection),
      stateInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      preJumpEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      preJumpInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      finalEqualityLagrangianPtr(new StateAugmentedLagrangianCollection),
      finalInequalityLagrangianPtr(new StateAugmentedLagrangianCollection),
      /* Misc. */
      preComputationPtr(new PreComputation),
      targetTrajectoriesPtr(nullptr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(const OptimalControlProblem& other)
    : /* Cost */
      costPtr(other.costPtr->clone()),
      stateCostPtr(other.stateCostPtr->clone()),
      preJumpCostPtr(other.preJumpCostPtr->clone()),
      finalCostPtr(other.finalCostPtr->clone()),
      /* Soft constraints */
      softConstraintPtr(other.softConstraintPtr->clone()),
      stateSoftConstraintPtr(other.stateSoftConstraintPtr->clone()),
      preJumpSoftConstraintPtr(other.preJumpSoftConstraintPtr->clone()),
      finalSoftConstraintPtr(other.finalSoftConstraintPtr->clone()),
      /* Equality constraints */
      equalityConstraintPtr(other.equalityConstraintPtr->clone()),
      stateEqualityConstraintPtr(other.stateEqualityConstraintPtr->clone()),
      preJumpEqualityConstraintPtr(other.preJumpEqualityConstraintPtr->clone()),
      finalEqualityConstraintPtr(other.finalEqualityConstraintPtr->clone()),
      /* Lagrangians */
      equalityLagrangianPtr(other.equalityLagrangianPtr->clone()),
      stateEqualityLagrangianPtr(other.stateEqualityLagrangianPtr->clone()),
      inequalityLagrangianPtr(other.inequalityLagrangianPtr->clone()),
      stateInequalityLagrangianPtr(other.stateInequalityLagrangianPtr->clone()),
      preJumpEqualityLagrangianPtr(other.preJumpEqualityLagrangianPtr->clone()),
      preJumpInequalityLagrangianPtr(other.preJumpInequalityLagrangianPtr->clone()),
      finalEqualityLagrangianPtr(other.finalEqualityLagrangianPtr->clone()),
      finalInequalityLagrangianPtr(other.finalInequalityLagrangianPtr->clone()),
      /* Misc. */
      preComputationPtr(other.preComputationPtr->clone()),
      targetTrajectoriesPtr(other.targetTrajectoriesPtr) {
  if (other.dynamicsPtr != nullptr) {
    dynamicsPtr.reset(other.dynamicsPtr->clone());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem& OptimalControlProblem::operator=(const OptimalControlProblem& rhs) {
  OptimalControlProblem tmp(rhs);
  swap(tmp);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OptimalControlProblem::swap(OptimalControlProblem& other) noexcept {
  /* Cost */
  costPtr.swap(other.costPtr);
  stateCostPtr.swap(other.stateCostPtr);
  preJumpCostPtr.swap(other.preJumpCostPtr);
  finalCostPtr.swap(other.finalCostPtr);

  /* Soft constraints */
  softConstraintPtr.swap(other.softConstraintPtr);
  stateSoftConstraintPtr.swap(other.stateSoftConstraintPtr);
  preJumpSoftConstraintPtr.swap(other.preJumpSoftConstraintPtr);
  finalSoftConstraintPtr.swap(other.finalSoftConstraintPtr);

  /* Equality constraints */
  equalityConstraintPtr.swap(other.equalityConstraintPtr);
  stateEqualityConstraintPtr.swap(other.stateEqualityConstraintPtr);
  preJumpEqualityConstraintPtr.swap(other.preJumpEqualityConstraintPtr);
  finalEqualityConstraintPtr.swap(other.finalEqualityConstraintPtr);

  /* Lagrangians */
  equalityLagrangianPtr.swap(other.equalityLagrangianPtr);
  stateEqualityLagrangianPtr.swap(other.stateEqualityLagrangianPtr);
  inequalityLagrangianPtr.swap(other.inequalityLagrangianPtr);
  stateInequalityLagrangianPtr.swap(other.stateInequalityLagrangianPtr);
  preJumpEqualityLagrangianPtr.swap(other.preJumpEqualityLagrangianPtr);
  preJumpInequalityLagrangianPtr.swap(other.preJumpInequalityLagrangianPtr);
  finalEqualityLagrangianPtr.swap(other.finalEqualityLagrangianPtr);
  finalInequalityLagrangianPtr.swap(other.finalInequalityLagrangianPtr);

  /* Dynamics */
  dynamicsPtr.swap(other.dynamicsPtr);

  /* Misc. */
  preComputationPtr.swap(other.preComputationPtr);
  std::swap(targetTrajectoriesPtr, other.targetTrajectoriesPtr);
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
bool extractIntermediateTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                                    const std::vector<MetricsCollection>& metricsCollTraj,
                                    std::vector<MetricsConstRef>& metricsTrajectory) {
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
bool extractPreJumpTermMetrics(const OptimalControlProblem& ocp, const std::string& name,
                               const std::vector<MetricsCollection>& metricsCollArray, std::vector<MetricsConstRef>& metricsArray) {
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
const Metrics* extractFinalTermMetrics(const OptimalControlProblem& ocp, const std::string& name, const MetricsCollection& metricsColl) {
  size_t index;
  if (ocp.finalEqualityLagrangianPtr->getTermIndex(name, index)) {
    return &metricsColl.stateEqLagrangian[index];

  } else if (ocp.finalInequalityLagrangianPtr->getTermIndex(name, index)) {
    return &metricsColl.stateIneqLagrangian[index];

  } else {
    return nullptr;
  }
}

}  // namespace ocs2
