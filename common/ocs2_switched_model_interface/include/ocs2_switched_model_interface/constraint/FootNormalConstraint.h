//
// Created by rgrandia on 29.04.20.
//

#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

/**
 * Implements the swing motion equality constraint in the normal direction of the terrain.
 * Active both in contact (to stabilize the position w.r.t. terrain) and in swing (to follow the trajectory in normal direction)
 *
 * The constraint is a hybrid position-velocity constraint formulated in task space:
 * A_p * position + A_v * velocity + b = 0
 */
class FootNormalConstraint : public ocs2::StateInputConstraint {
 public:
  FootNormalConstraint(int legNumber, scalar_t positionGain);

  FootNormalConstraint* clone() const override;

  bool isActive(scalar_t time) const override { return true; }

  size_t getNumConstraints(scalar_t time) const override { return 1; }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const ocs2::PreComputation& preComp) const override;

 private:
  FootNormalConstraint(const FootNormalConstraint& rhs);

  const int legNumber_;
  const scalar_t positionGain_;
};

}  // namespace switched_model
