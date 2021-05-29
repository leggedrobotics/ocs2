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
    : preComputationPtr(new PreComputation),
      equalityConstraintPtr(new StateInputConstraintCollection),
      stateEqualityConstraintPtr(new StateConstraintCollection),
      inequalityConstraintPtr(new StateInputConstraintCollection),
      preJumpEqualityConstraintPtr(new StateConstraintCollection),
      finalEqualityConstraintPtr(new StateConstraintCollection),
      softConstraintPtr(new StateInputCostCollection),
      stateSoftConstraintPtr(new StateCostCollection),
      preJumpSoftConstraintPtr(new StateCostCollection),
      finalSoftConstraintPtr(new StateCostCollection),
      costPtr(new StateInputCostCollection),
      stateCostPtr(new StateCostCollection),
      preJumpCostPtr(new StateCostCollection),
      finalCostPtr(new StateCostCollection) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(const OptimalControlProblem& other)
    : equalityConstraintPtr(other.equalityConstraintPtr->clone()),
      stateEqualityConstraintPtr(other.stateEqualityConstraintPtr->clone()),
      inequalityConstraintPtr(other.inequalityConstraintPtr->clone()),
      preJumpEqualityConstraintPtr(other.preJumpEqualityConstraintPtr->clone()),
      finalEqualityConstraintPtr(other.finalEqualityConstraintPtr->clone()),
      softConstraintPtr(other.softConstraintPtr->clone()),
      stateSoftConstraintPtr(other.stateSoftConstraintPtr->clone()),
      preJumpSoftConstraintPtr(other.preJumpSoftConstraintPtr->clone()),
      finalSoftConstraintPtr(other.finalSoftConstraintPtr->clone()),
      costPtr(other.costPtr->clone()),
      stateCostPtr(other.stateCostPtr->clone()),
      preJumpCostPtr(other.preJumpCostPtr->clone()),
      finalCostPtr(other.finalCostPtr->clone()),
      preComputationPtr(other.preComputationPtr->clone()) {
  // validtity check
  if (other.dynamicsPtr == nullptr) {
    throw std::runtime_error("[OptimalControlProblem] dynamicsPtr is not set");
  }
  dynamicsPtr.reset(other.dynamicsPtr->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(OptimalControlProblem&& other) noexcept
    : dynamicsPtr(std::move(other.dynamicsPtr)),
      equalityConstraintPtr(std::move(other.equalityConstraintPtr)),
      stateEqualityConstraintPtr(std::move(other.stateEqualityConstraintPtr)),
      inequalityConstraintPtr(std::move(other.inequalityConstraintPtr)),
      preJumpEqualityConstraintPtr(std::move(other.preJumpEqualityConstraintPtr)),
      finalEqualityConstraintPtr(std::move(other.finalEqualityConstraintPtr)),
      softConstraintPtr(std::move(other.softConstraintPtr)),
      stateSoftConstraintPtr(std::move(other.stateSoftConstraintPtr)),
      preJumpSoftConstraintPtr(std::move(other.preJumpSoftConstraintPtr)),
      finalSoftConstraintPtr(std::move(other.finalSoftConstraintPtr)),
      costPtr(std::move(other.costPtr)),
      stateCostPtr(std::move(other.stateCostPtr)),
      preJumpCostPtr(std::move(other.preJumpCostPtr)),
      finalCostPtr(std::move(other.finalCostPtr)),
      preComputationPtr(std::move(other.preComputationPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem& OptimalControlProblem::operator=(const OptimalControlProblem& rhs) {
  *this = OptimalControlProblem(rhs);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem& OptimalControlProblem::operator=(OptimalControlProblem&& rhs) {
  dynamicsPtr = std::move(rhs.dynamicsPtr);
  equalityConstraintPtr = std::move(rhs.equalityConstraintPtr);
  stateEqualityConstraintPtr = std::move(rhs.stateEqualityConstraintPtr);
  inequalityConstraintPtr = std::move(rhs.inequalityConstraintPtr);
  preJumpEqualityConstraintPtr = std::move(rhs.preJumpEqualityConstraintPtr);
  finalEqualityConstraintPtr = std::move(rhs.finalEqualityConstraintPtr);
  softConstraintPtr = std::move(rhs.softConstraintPtr);
  stateSoftConstraintPtr = std::move(rhs.stateSoftConstraintPtr);
  preJumpSoftConstraintPtr = std::move(rhs.preJumpSoftConstraintPtr);
  finalSoftConstraintPtr = std::move(rhs.finalSoftConstraintPtr);
  costPtr = std::move(rhs.costPtr);
  stateCostPtr = std::move(rhs.stateCostPtr);
  preJumpCostPtr = std::move(rhs.preJumpCostPtr);
  finalCostPtr = std::move(rhs.finalCostPtr);
  preComputationPtr = std::move(rhs.preComputationPtr);
  return *this;
}

}  // namespace ocs2
