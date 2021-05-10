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

#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/ZeroStateCost.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase::CostFunctionBase(std::unique_ptr<StateInputCost> costPtr, std::unique_ptr<StateCost> preJumpCostPtr,
                                   std::unique_ptr<StateCost> finalCostPtr, std::shared_ptr<PreComputation> preCompPtr)
    : costPtr_(std::move(costPtr)),
      preJumpCostPtr_(std::move(preJumpCostPtr)),
      finalCostPtr_(std::move(finalCostPtr)),
      preCompPtr_(std::move(preCompPtr)) {
  if (costPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costPtr is not set");
  }
  if (finalCostPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] finalCostPtr is not set");
  }
  if (preJumpCostPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] preJumpCostPtr is not set");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase::CostFunctionBase(std::unique_ptr<StateInputCost> costPtr, std::unique_ptr<StateCost> finalCostPtr,
                                   std::shared_ptr<PreComputation> preCompPtr)
    : CostFunctionBase(std::move(costPtr), std::unique_ptr<StateCost>(new ZeroStateCost), std::move(finalCostPtr), std::move(preCompPtr)) {
  // delegate constructor with zero pre-jump cost.
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase::CostFunctionBase(const CostFunctionBase& other, std::shared_ptr<PreComputation> preCompPtr)
    : costPtr_(other.costPtr_->clone()), finalCostPtr_(other.finalCostPtr_->clone()), preJumpCostPtr_(other.preJumpCostPtr_->clone()) {
  preCompPtr_ = std::move(preCompPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase* CostFunctionBase::clone() const {
  return new CostFunctionBase(*this, std::shared_ptr<PreComputation>(preCompPtr_->clone()));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CostFunctionBase* CostFunctionBase::clone(std::shared_ptr<PreComputation> preCompPtr) const {
  return new CostFunctionBase(*this, std::move(preCompPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBase::getValue(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost, t, x, u);
  }
  return costPtr_->getValue(t, x, u, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBase::getPreJumpValue(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost, t, x);
  }
  return preJumpCostPtr_->getValue(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t CostFunctionBase::getFinalValue(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost, t, x);
  }
  return finalCostPtr_->getValue(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionBase::getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->request(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x, u);
  }
  return costPtr_->getQuadraticApproximation(t, x, u, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionBase::getPreJumpQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestPreJump(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return preJumpCostPtr_->getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation CostFunctionBase::getFinalQuadraticApproximation(scalar_t t, const vector_t& x) {
  if (costDesiredTrajectoriesPtr_ == nullptr) {
    throw std::runtime_error("[CostFunctionBase] costDesiredTrajectoriesPtr_ is not set.");
  }
  if (preCompPtr_ != nullptr) {
    preCompPtr_->requestFinal(PreComputation::Request::Cost | PreComputation::Request::Approximation, t, x);
  }
  return finalCostPtr_->getQuadraticApproximation(t, x, *costDesiredTrajectoriesPtr_, preCompPtr_.get());
}

}  // namespace ocs2
