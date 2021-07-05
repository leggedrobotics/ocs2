//
// Created by rgrandia on 05.07.21.
//

#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"

namespace switched_model {

ZeroForceConstraint::ZeroForceConstraint() : ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear) {}

ZeroForceConstraint* ZeroForceConstraint::clone() const {
  return new ZeroForceConstraint(*this);
}

void ZeroForceConstraint::setContactFlags(const contact_flag_t& contactFlags) {
  contactFlags_ = contactFlags;
}

size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const {
  return 3 * numberOfOpenContacts(contactFlags_);
};

vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input) const {
  vector_t constraints(getNumConstraints(time));

  int constraintIdx = 0;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (!contactFlags_[leg]) {
      const int legStartIdx = 3 * leg;
      constraints.segment<3>(constraintIdx) = input.segment(legStartIdx, 3);
      constraintIdx += 3;
    }
  }

  return constraints;
};

VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                              const vector_t& input) const {
  auto linearApproximation = VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.rows(), input.rows());

  int constraintIdx = 0;
  for (int leg = 0; leg < NUM_CONTACT_POINTS; ++leg) {
    if (!contactFlags_[leg]) {
      const int legStartIdx = 3 * leg;
      linearApproximation.f.segment<3>(constraintIdx) = input.segment(legStartIdx, 3);
      linearApproximation.dfdu.block<3, 3>(constraintIdx, legStartIdx).setIdentity();
      constraintIdx += 3;
    }
  }

  return linearApproximation;
}

}  // namespace switched_model