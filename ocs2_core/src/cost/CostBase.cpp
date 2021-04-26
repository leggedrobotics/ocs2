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

#include <ocs2_core/cost/CostBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostBase::CostBase(PreComputation* preCompPtr) : preCompPtr_(preCompPtr) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getValue(scalar_t t, const vector_t& x, const vector_t& u) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost, t, x, u);
  }
  return getValue(t, x, u, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getPreJumpValue(scalar_t t, const vector_t& x, const vector_t& u) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost, t, x);
  }
  return getPreJumpValue(t, x, u, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getFinalValue(scalar_t t, const vector_t& x) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost, t, x);
  }
  return getFinalValue(t, x, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x, u);
  }
  return getQuadraticApproximation(t, x, u, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getPreJumpQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return getPreJumpQuadraticApproximation(t, x, u, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getFinalQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return getFinalQuadraticApproximation(t, x, preCompPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CostBase::setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) {
  costDesiredTrajectoriesPtr_ = costDesiredTrajectoriesPtr;
}

}  // namespace ocs2
