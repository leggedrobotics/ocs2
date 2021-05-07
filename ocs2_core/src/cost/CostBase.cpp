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
CostBase::CostBase(std::unique_ptr<StateInputCost> costPtr, std::unique_ptr<StateCost> finalCostPtr,
                   std::unique_ptr<StateCost> preJumpCostPtr, std::shared_ptr<PreComputation> preCompPtr)
    : costPtr_(std::move(costPtr)),
      finalCostPtr_(std::move(finalCostPtr)),
      preJumpCostPtr_(std::move(preJumpCostPtr)),
      preCompPtr_(std::move(preCompPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostBase::CostBase(const CostBase& other, std::shared_ptr<PreComputation> preCompPtr)
    : costPtr_(other.costPtr_->clone()), finalCostPtr_(other.finalCostPtr_->clone()), preJumpCostPtr_(other.preJumpCostPtr_->clone()) {
  preCompPtr_ = std::move(preCompPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostBase* CostBase::clone() const {
  return new CostBase(*this, std::shared_ptr<PreComputation>(preCompPtr_->clone()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostBase* CostBase::clone(std::shared_ptr<PreComputation> preCompPtr) const {
  return new CostBase(*this, std::move(preCompPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getValue(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost, t, x, u);
  }
  return costPtr_->getValue(t, x, u, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getPreJumpValue(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost, t, x);
  }
  return preJumpCostPtr_->getValue(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostBase::getFinalValue(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost, t, x);
  }
  return finalCostPtr_->getValue(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x, u);
  }
  return costPtr_->getQuadraticApproximation(t, x, u, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getPreJumpQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return preJumpCostPtr_->getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostBase::getFinalQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return finalCostPtr_->getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

}  // namespace ocs2
