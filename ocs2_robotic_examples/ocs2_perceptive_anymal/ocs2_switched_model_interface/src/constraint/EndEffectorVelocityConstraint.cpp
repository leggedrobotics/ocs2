//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"

#include "ocs2_switched_model_interface/core/SwitchedModelPrecomputation.h"

namespace switched_model {

EndEffectorVelocityConstraint::EndEffectorVelocityConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : StateInputConstraint(ocs2::ConstraintOrder::Linear), legNumber_(legNumber), modeScheduleManager_(&modeScheduleManager) {}

EndEffectorVelocityConstraint* EndEffectorVelocityConstraint::clone() const {
  return new EndEffectorVelocityConstraint(*this);
};

bool EndEffectorVelocityConstraint::isActive(scalar_t time) const {
  return modeScheduleManager_->getContactFlags(time)[legNumber_];
}

size_t EndEffectorVelocityConstraint::getNumConstraints(scalar_t time) const {
  return 2;
}

vector_t EndEffectorVelocityConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                 const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto tangentBasis = tangentialBasisFromSurfaceNormal(switchedModelPreComp.getSurfaceNormalInOriginFrame(legNumber_));

  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);
  return tangentBasis * o_footVelocity;
}

VectorFunctionLinearApproximation EndEffectorVelocityConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                        const vector_t& input,
                                                                                        const ocs2::PreComputation& preComp) const {
  const auto& switchedModelPreComp = ocs2::cast<SwitchedModelPreComputation>(preComp);
  const auto tangentBasis = tangentialBasisFromSurfaceNormal(switchedModelPreComp.getSurfaceNormalInOriginFrame(legNumber_));

  const auto& o_footVelocity = switchedModelPreComp.footVelocityInOriginFrame(legNumber_);
  const auto& o_footVelocityDerivative = switchedModelPreComp.footVelocityInOriginFrameDerivative(legNumber_);

  VectorFunctionLinearApproximation constraint;
  constraint.f.noalias() = tangentBasis * o_footVelocity;
  constraint.dfdx.noalias() = tangentBasis * o_footVelocityDerivative.dfdx;
  constraint.dfdu.noalias() = tangentBasis * o_footVelocityDerivative.dfdu;
  return constraint;
}

}  // namespace switched_model