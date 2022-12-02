/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "ocs2_perceptive/end_effector/EndEffectorDistanceConstraint.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight,
                                                             std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      weight_(weight),
      kinematicsPtr_(std::move(kinematicsPtr)),
      clearances_(kinematicsPtr_->getIds().size(), 0.0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(size_t stateDim, scalar_t weight,
                                                             std::unique_ptr<EndEffectorKinematics<scalar_t>> kinematicsPtr,
                                                             scalar_array_t clearances)
    : StateConstraint(ConstraintOrder::Linear),
      stateDim_(stateDim),
      weight_(weight),
      kinematicsPtr_(std::move(kinematicsPtr)),
      clearances_(std::move(clearances)) {
  if (clearances_.size() != kinematicsPtr_->getIds().size()) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] clearances.size() != kinematicsPtr->getIds().size()");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorDistanceConstraint::EndEffectorDistanceConstraint(const EndEffectorDistanceConstraint& other)
    : StateConstraint(other),
      stateDim_(other.stateDim_),
      weight_(other.weight_),
      kinematicsPtr_(other.kinematicsPtr_->clone()),
      clearances_(other.clearances_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(const DistanceTransformInterface& distanceTransform) {
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(scalar_t clearance, const DistanceTransformInterface& distanceTransform) {
  clearances_.assign(kinematicsPtr_->getIds().size(), clearance);
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorDistanceConstraint::set(const scalar_array_t& clearances, const DistanceTransformInterface& distanceTransform) {
  assert(clearances.size() == kinematicsPtr_->getIds().size());
  clearances_ = clearances;
  distanceTransformPtr_ = &distanceTransform;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorDistanceConstraint::getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const {
  if (distanceTransformPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
  }

  const auto numEEs = kinematicsPtr_->getIds().size();
  const auto eePositions = kinematicsPtr_->getPosition(state);

  vector_t g(numEEs);
  for (size_t i = 0; i < numEEs; i++) {
    g(i) = weight_ * (distanceTransformPtr_->getValue(eePositions[i]) - clearances_[i]);
  }  // end of i loop

  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorDistanceConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const PreComputation& preComp) const {
  if (distanceTransformPtr_ == nullptr) {
    throw std::runtime_error("[EndEffectorDistanceConstraint] First, set the distance-transform by calling set()!");
  }

  const auto numEEs = kinematicsPtr_->getIds().size();
  const auto eePosLinApprox = kinematicsPtr_->getPositionLinearApproximation(state);

  VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(numEEs, stateDim_, 0);
  for (size_t i = 0; i < numEEs; i++) {
    const auto distanceValueGradient = distanceTransformPtr_->getLinearApproximation(eePosLinApprox[i].f);
    approx.f(i) = weight_ * (distanceValueGradient.first - clearances_[i]);
    approx.dfdx.row(i).noalias() = weight_ * (distanceValueGradient.second.transpose() * eePosLinApprox[i].dfdx);
  }  // end of i loop

  return approx;
}

}  // namespace ocs2
