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

#include "ocs2_oc/synchronized_module/SolverObserver.h"

#include "ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h"

namespace ocs2 {

namespace {
std::string toString(SolverObserver::Type type) {
  switch (type) {
    case SolverObserver::Type::Final:
      return "Final";
    case SolverObserver::Type::PreJump:
      return "PreJump";
    case SolverObserver::Type::Intermediate:
      return "Intermediate";
    default:
      throw std::runtime_error("[SolverObserver::toString] undefined type!");
  }
}
}  // unnamed namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::extractTermConstraint(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                           const ProblemMetrics& problemMetrics) {
  if (!constraintCallback_ || primalSolution.timeTrajectory_.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* termConstraintPtr = extractFinalTermConstraint(ocp, termName_, problemMetrics.final);
      termIsFound = termConstraintPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{primalSolution.timeTrajectory_.back()};
        const std::vector<std::reference_wrapper<const vector_t>> termConstraintArray{*termConstraintPtr};
        constraintCallback_(timeArray, termConstraintArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<std::reference_wrapper<const vector_t>> termConstraintArray;
      termIsFound = extractPreJumpTermConstraint(ocp, termName_, problemMetrics.preJumps, termConstraintArray);
      if (termIsFound) {
        scalar_array_t timeArray(primalSolution.postEventIndices_.size());
        std::transform(primalSolution.postEventIndices_.cbegin(), primalSolution.postEventIndices_.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return primalSolution.timeTrajectory_[postInd - 1]; });
        constraintCallback_(timeArray, termConstraintArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<std::reference_wrapper<const vector_t>> termConstraintArray;
      termIsFound = extractIntermediateTermConstraint(ocp, termName_, problemMetrics.intermediates, termConstraintArray);
      if (termIsFound) {
        constraintCallback_(primalSolution.timeTrajectory_, termConstraintArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::extractTermConstraint] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::extractTermConstraint] term (" + termName_ + ") does not exist in " + toString(type_) +
                             "-time constraint collections!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::extractTermLagrangianMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                                  const ProblemMetrics& problemMetrics) {
  if (!lagrangianCallback_ || primalSolution.timeTrajectory_.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* lagrangianMetricsPtr = extractFinalTermLagrangianMetrics(ocp, termName_, problemMetrics.final);
      termIsFound = lagrangianMetricsPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{primalSolution.timeTrajectory_.back()};
        const std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray{*lagrangianMetricsPtr};
        lagrangianCallback_(timeArray, termLagrangianMetricsArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray;
      termIsFound = extractPreJumpTermLagrangianMetrics(ocp, termName_, problemMetrics.preJumps, termLagrangianMetricsArray);
      if (termIsFound) {
        scalar_array_t timeArray(primalSolution.postEventIndices_.size());
        std::transform(primalSolution.postEventIndices_.cbegin(), primalSolution.postEventIndices_.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return primalSolution.timeTrajectory_[postInd - 1]; });
        lagrangianCallback_(timeArray, termLagrangianMetricsArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<LagrangianMetricsConstRef> termLagrangianMetricsArray;
      termIsFound = extractIntermediateTermLagrangianMetrics(ocp, termName_, problemMetrics.intermediates, termLagrangianMetricsArray);
      if (termIsFound) {
        lagrangianCallback_(primalSolution.timeTrajectory_, termLagrangianMetricsArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::extractTermLagrangianMetrics] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::extractTermLagrangianMetrics] term (" + termName_ + ") does not exist in " +
                             toString(type_) + "-time Lagrangian collections!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverObserver::extractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution) {
  if (!multiplierCallback_ || dualSolution.timeTrajectory.empty()) {
    return;
  }

  bool termIsFound = true;
  switch (type_) {
    case Type::Final: {
      const auto* multiplierPtr = extractFinalTermMultiplier(ocp, termName_, dualSolution.final);
      termIsFound = multiplierPtr != nullptr;
      if (termIsFound) {
        const scalar_array_t timeArray{dualSolution.timeTrajectory.back()};
        const std::vector<MultiplierConstRef> termMultiplierArray{*multiplierPtr};
        multiplierCallback_(timeArray, termMultiplierArray);
      }
      break;
    }
    case Type::PreJump: {
      std::vector<MultiplierConstRef> termMultiplierArray;
      termIsFound = extractPreJumpTermMultiplier(ocp, termName_, dualSolution.preJumps, termMultiplierArray);
      if (termIsFound) {
        scalar_array_t timeArray(dualSolution.postEventIndices.size());
        std::transform(dualSolution.postEventIndices.cbegin(), dualSolution.postEventIndices.cend(), timeArray.begin(),
                       [&](size_t postInd) -> scalar_t { return dualSolution.timeTrajectory[postInd - 1]; });
        multiplierCallback_(timeArray, termMultiplierArray);
      }
      break;
    }
    case Type::Intermediate: {
      std::vector<MultiplierConstRef> termMultiplierArray;
      termIsFound = extractIntermediateTermMultiplier(ocp, termName_, dualSolution.intermediates, termMultiplierArray);
      if (termIsFound) {
        multiplierCallback_(dualSolution.timeTrajectory, termMultiplierArray);
      }
      break;
    }
    default:
      throw std::runtime_error("[SolverObserver::extractTermMultipliers] undefined type!");
  }

  if (!termIsFound) {
    throw std::runtime_error("[SolverObserver::extractTermMultipliers] term (" + termName_ + ") does not exist in " + toString(type_) +
                             "-time Lagrangian collections!");
  }
}

}  // namespace ocs2
