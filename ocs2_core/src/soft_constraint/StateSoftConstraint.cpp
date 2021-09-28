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

#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr,
                                         std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(std::unique_ptr<StateConstraint> constraintPtr, std::unique_ptr<PenaltyBase> penaltyFunction)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyFunction)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint::StateSoftConstraint(const StateSoftConstraint& other)
    : StateCost(other), constraintPtr_(other.constraintPtr_->clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateSoftConstraint* StateSoftConstraint::clone() const {
  return new StateSoftConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateSoftConstraint::isActive(scalar_t time) const {
  return constraintPtr_->isActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t StateSoftConstraint::getValue(scalar_t time, const vector_t& state, const TargetTrajectories&,
                                       const PreComputation& preComp) const {
  return penalty_.getValue(time, constraintPtr_->getValue(time, state, preComp));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateSoftConstraint::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                    const TargetTrajectories&,
                                                                                    const PreComputation& preComp) const {
  switch (constraintPtr_->getOrder()) {
    case ConstraintOrder::Linear:
      return penalty_.getQuadraticApproximation(time, constraintPtr_->getLinearApproximation(time, state, preComp));
    case ConstraintOrder::Quadratic:
      return penalty_.getQuadraticApproximation(time, constraintPtr_->getQuadraticApproximation(time, state, preComp));
    default:
      throw std::runtime_error("[StateSoftConstraint] Unknown constraint Order");
  }
}

}  // namespace ocs2
