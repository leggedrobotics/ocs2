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
OptimalControlProblem::OptimalControlProblem() : preComputation(new PreComputation) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(const OptimalControlProblem& other)
    : equalityConstraint(other.equalityConstraint),
      stateEqualityConstraint(other.stateEqualityConstraint),
      inequalityConstraint(other.inequalityConstraint),
      preJumpEqualityConstraint(other.preJumpEqualityConstraint),
      finalEqualityConstraint(other.finalEqualityConstraint),
      softConstraint(other.softConstraint),
      preJumpSoftConstraint(other.preJumpSoftConstraint),
      finalSoftConstraint(other.finalSoftConstraint),
      cost(other.cost),
      stateCost(other.stateCost),
      preJumpCost(other.preJumpCost),
      finalCost(other.finalCost),
      preComputation(other.preComputation->clone()) {
  // validtity check
  if (other.dynamics == nullptr) {
    throw std::runtime_error("[OptimalControlProblem] system dynamics pointer is not set");
  }
  if (other.preComputation == nullptr) {
    throw std::runtime_error("[OptimalControlProblem] preComputation pointer is not set");
  }
  dynamics.reset(other.dynamics->clone());
  preComputation.reset(other.preComputation->clone());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OptimalControlProblem::OptimalControlProblem(OptimalControlProblem&& other) noexcept
    : dynamics(std::move(other.dynamics)),
      equalityConstraint(std::move(other.equalityConstraint)),
      stateEqualityConstraint(std::move(other.stateEqualityConstraint)),
      inequalityConstraint(std::move(other.inequalityConstraint)),
      preJumpEqualityConstraint(std::move(other.preJumpEqualityConstraint)),
      finalEqualityConstraint(std::move(other.finalEqualityConstraint)),
      softConstraint(std::move(other.softConstraint)),
      preJumpSoftConstraint(std::move(other.preJumpSoftConstraint)),
      finalSoftConstraint(std::move(other.finalSoftConstraint)),
      cost(std::move(other.cost)),
      stateCost(std::move(other.stateCost)),
      preJumpCost(std::move(other.preJumpCost)),
      finalCost(std::move(other.finalCost)),
      preComputation(std::move(other.preComputation)) {}

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
  dynamics = std::move(rhs.dynamics);
  equalityConstraint = std::move(rhs.equalityConstraint);
  stateEqualityConstraint = std::move(rhs.stateEqualityConstraint);
  inequalityConstraint = std::move(rhs.inequalityConstraint);
  preJumpEqualityConstraint = std::move(rhs.preJumpEqualityConstraint);
  finalEqualityConstraint = std::move(rhs.finalEqualityConstraint);
  softConstraint = std::move(rhs.softConstraint);
  preJumpSoftConstraint = std::move(rhs.preJumpSoftConstraint);
  finalSoftConstraint = std::move(rhs.finalSoftConstraint);
  cost = std::move(rhs.cost);
  stateCost = std::move(rhs.stateCost);
  preJumpCost = std::move(rhs.preJumpCost);
  finalCost = std::move(rhs.finalCost);
  preComputation = std::move(rhs.preComputation);
  return *this;
}

}  // namespace ocs2
