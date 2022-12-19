//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

ZeroForceConstraint::ZeroForceConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear),
      legStartIdx_(3 * legNumber),
      legNumber_(legNumber),
      modeScheduleManager_(&modeScheduleManager) {}

ZeroForceConstraint* ZeroForceConstraint::clone() const {
  return new ZeroForceConstraint(*this);
}

bool ZeroForceConstraint::isActive(scalar_t time) const {
  return !modeScheduleManager_->getContactFlags(time)[legNumber_];
}

size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const {
  return 3;
};

vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                       const ocs2::PreComputation& preComp) const {
  return input.segment(legStartIdx_, 3);
};

VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                              const ocs2::PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = input.segment(legStartIdx_, 3);
  linearApproximation.dfdx.setZero(3, state.rows());
  linearApproximation.dfdu.setZero(3, input.rows());
  linearApproximation.dfdu.block<3, 3>(0, legStartIdx_).setIdentity();
  return linearApproximation;
}

}  // namespace switched_model