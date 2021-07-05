//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

ZeroForceConstraint::ZeroForceConstraint(int legNumber)
    : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear), legStartIdx_(3 * legNumber) {}

ZeroForceConstraint* ZeroForceConstraint::clone() const {
  return new ZeroForceConstraint(*this);
}

size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const {
  return 3;
};

vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  return input.segment(legStartIdx_, 3);
};

VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const vector_t& input) const {
  VectorFunctionLinearApproximation linearApproximation;
  linearApproximation.f = input.segment(legStartIdx_, 3);
  linearApproximation.dfdx.setZero(3, state.rows());
  linearApproximation.dfdu.setZero(3, input.rows());
  linearApproximation.dfdu.block<3, 3>(0, legStartIdx_).setIdentity();
  return linearApproximation;
}

}  // namespace switched_model