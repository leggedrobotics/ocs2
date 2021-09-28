//
// Created by rgrandia on 29.04.20.
//

#include "ocs2_switched_model_interface/constraint/FootNormalConstraint.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

namespace switched_model {

FootNormalConstraint::FootNormalConstraint(int legNumber)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), legNumber_(legNumber) {}

FootNormalConstraint::FootNormalConstraint(const FootNormalConstraint& rhs) : ocs2::StateInputConstraint(rhs), legNumber_(rhs.legNumber_) {}

FootNormalConstraint* FootNormalConstraint::clone() const {
  return new FootNormalConstraint(*this);
}

vector_t FootNormalConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                        const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& footPhase = switchedModelPreComp.getFootPhase(legNumber_);
  const auto& normalConstraint = footPhase.getFootNormalConstraintInWorldFrame(time);

  const auto& o_footPosition = switchedModelPreComp.footPositionInOriginFrame(legNumber_);
  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);

  return (vector_t(1) << normalConstraint.positionMatrix.dot(o_footPosition) + normalConstraint.velocityMatrix.dot(o_footVelocity) +
                             normalConstraint.constant)
      .finished();
}

VectorFunctionLinearApproximation FootNormalConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                               const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto& footPhase = switchedModelPreComp.getFootPhase(legNumber_);
  const auto& normalConstraint = footPhase.getFootNormalConstraintInWorldFrame(time);

  const auto& o_footPosition = switchedModelPreComp.footPositionInOriginFrame(legNumber_);
  const auto& o_footPositionDerivative = switchedModelPreComp.footPositionInOriginFrameStateDerivative(legNumber_);
  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);
  const auto& o_footVelocityDerivative = switchedModelPreComp.footVelocityInOriginFrameDerivative(legNumber_);

  VectorFunctionLinearApproximation constraint;
  // Constant
  constraint.f = (vector_t(1) << normalConstraint.positionMatrix.dot(o_footPosition) + normalConstraint.velocityMatrix.dot(o_footVelocity) +
                                     normalConstraint.constant)
                     .finished();
  // State derivative
  constraint.dfdx.noalias() = normalConstraint.positionMatrix * o_footPositionDerivative;
  constraint.dfdx.noalias() += normalConstraint.velocityMatrix * o_footVelocityDerivative.dfdx;

  // Input derivative
  constraint.dfdu.noalias() = normalConstraint.velocityMatrix * o_footVelocityDerivative.dfdu;
  return constraint;
}

}  // namespace switched_model
