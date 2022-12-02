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

#include "ocs2_oc/synchronized_module/AugmentedLagrangianObserver.h"

#include "ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AugmentedLagrangianObserver::extractTermMetrics(const OptimalControlProblem& ocp, const PrimalSolution& primalSolution,
                                                     const ProblemMetrics& problemMetrics) {
  // search intermediate
  if (extractIntermediateTermMetrics(ocp, termName_, problemMetrics.intermediates, termMetricsArray_)) {
    if (!primalSolution.timeTrajectory_.empty() && metricsCallback_ != nullptr) {
      metricsCallback_(primalSolution.timeTrajectory_, termMetricsArray_);
    }
  }

  // search pre_jump
  else if (extractPreJumpTermMetrics(ocp, termName_, problemMetrics.preJumps, termMetricsArray_)) {
    if (!primalSolution.postEventIndices_.empty() && metricsCallback_ != nullptr) {
      scalar_array_t timeArray(primalSolution.postEventIndices_.size());
      std::transform(primalSolution.postEventIndices_.cbegin(), primalSolution.postEventIndices_.cend(), timeArray.begin(),
                     [&](size_t postInd) -> scalar_t { return primalSolution.timeTrajectory_[postInd - 1]; });
      metricsCallback_(timeArray, termMetricsArray_);
    }
  }

  // search final
  else {
    const LagrangianMetrics* metricsPtr = extractFinalTermMetrics(ocp, termName_, problemMetrics.final);
    if (metricsPtr != nullptr) {
      if (!primalSolution.timeTrajectory_.empty() && metricsCallback_ != nullptr) {
        const scalar_array_t timeArray{primalSolution.timeTrajectory_.back()};
        termMetricsArray_.push_back(*metricsPtr);
        metricsCallback_(timeArray, termMetricsArray_);
      }

    } else {
      throw std::runtime_error("[AugmentedLagrangianObserver::extractTermsMetrics] Term (" + termName_ +
                               ") does not exist in the Lagrangian collections!");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AugmentedLagrangianObserver::extractTermMultipliers(const OptimalControlProblem& ocp, const DualSolution& dualSolution) {
  // search intermediate
  if (extractIntermediateTermMultiplier(ocp, termName_, dualSolution.intermediates, termMultiplierArray_)) {
    if (!dualSolution.timeTrajectory.empty() && multiplierCallback_ != nullptr) {
      multiplierCallback_(dualSolution.timeTrajectory, termMultiplierArray_);
    }
  }

  // search pre_jump
  else if (extractPreJumpTermMultiplier(ocp, termName_, dualSolution.preJumps, termMultiplierArray_)) {
    if (!dualSolution.postEventIndices.empty() && multiplierCallback_ != nullptr) {
      scalar_array_t timeArray(dualSolution.postEventIndices.size());
      std::transform(dualSolution.postEventIndices.cbegin(), dualSolution.postEventIndices.cend(), timeArray.begin(),
                     [&](size_t postInd) -> scalar_t { return dualSolution.timeTrajectory[postInd - 1]; });
      multiplierCallback_(timeArray, termMultiplierArray_);
    }
  }

  // search final
  else {
    const Multiplier* multiplierPtr = extractFinalTermMultiplier(ocp, termName_, dualSolution.final);
    if (multiplierPtr != nullptr) {
      if (!dualSolution.timeTrajectory.empty() && multiplierCallback_ != nullptr) {
        const scalar_array_t timeArray{dualSolution.timeTrajectory.back()};
        termMultiplierArray_.push_back(*multiplierPtr);
        multiplierCallback_(timeArray, termMultiplierArray_);
      }

    } else {
      throw std::runtime_error("[AugmentedLagrangianObserver::extractTermMultipliers] Term (" + termName_ +
                               ") does not exist in the Lagrangian collections!");
    }
  }
}

}  // namespace ocs2
