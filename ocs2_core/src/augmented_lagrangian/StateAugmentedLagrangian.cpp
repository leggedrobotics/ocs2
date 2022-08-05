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

#include <ocs2_core/augmented_lagrangian/StateAugmentedLagrangian.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr,
                                                   std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(std::unique_ptr<StateConstraint> constraintPtr,
                                                   std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr)
    : constraintPtr_(std::move(constraintPtr)), penalty_(std::move(penaltyPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian::StateAugmentedLagrangian(const StateAugmentedLagrangian& other)
    : StateAugmentedLagrangianInterface(other), constraintPtr_(other.constraintPtr_->clone()), penalty_(other.penalty_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateAugmentedLagrangian* StateAugmentedLagrangian::clone() const {
  return new StateAugmentedLagrangian(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateAugmentedLagrangian::isActive(scalar_t time) const {
  return constraintPtr_->isActive(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t StateAugmentedLagrangian::getNumConstraints(scalar_t time) const {
  return constraintPtr_->getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LagrangianMetrics StateAugmentedLagrangian::getValue(scalar_t time, const vector_t& state, const Multiplier& multiplier,
                                                     const PreComputation& preComp) const {
  const auto h = constraintPtr_->getValue(time, state, preComp);
  const auto p = multiplier.penalty * penalty_.getValue(time, h, &multiplier.lagrangian);
  return {p, h};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation StateAugmentedLagrangian::getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                                         const Multiplier& multiplier,
                                                                                         const PreComputation& preComp) const {
  switch (constraintPtr_->getOrder()) {
    case ConstraintOrder::Linear:
      return multiplier.penalty *
             penalty_.getQuadraticApproximation(time, constraintPtr_->getLinearApproximation(time, state, preComp), &multiplier.lagrangian);
    case ConstraintOrder::Quadratic:
      return multiplier.penalty * penalty_.getQuadraticApproximation(time, constraintPtr_->getQuadraticApproximation(time, state, preComp),
                                                                     &multiplier.lagrangian);
    default:
      throw std::runtime_error("[StateAugmentedLagrangian] Unknown constraint Order");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<Multiplier, scalar_t> StateAugmentedLagrangian::updateLagrangian(scalar_t time, const vector_t& /*state*/,
                                                                           const vector_t& constraint, const Multiplier& multiplier) const {
  const Multiplier updatedMultiplier{multiplier.penalty, penalty_.updateMultipliers(time, constraint, multiplier.lagrangian)};
  const auto penalty = updatedMultiplier.penalty * penalty_.getValue(time, constraint, &updatedMultiplier.lagrangian);
  return {updatedMultiplier, penalty};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Multiplier StateAugmentedLagrangian::initializeLagrangian(scalar_t time) const {
  return {1.0, penalty_.initializeMultipliers(getNumConstraints(time))};
}

}  // namespace ocs2
