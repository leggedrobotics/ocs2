#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

namespace switched_model {

/**
 * Implements the constraint that the velocity of the foot w.r.t. the world, that is tangent to the surface normal,
 * must be zero for a foot in contact:
 *
 * Tangent(n) * footVelocity = 0
 *
 * Uses the precomputed foot velocity.
 */
class EndEffectorVelocityConstraint final : public ocs2::StateInputConstraint {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EndEffectorVelocityConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager);

  ~EndEffectorVelocityConstraint() override = default;

  EndEffectorVelocityConstraint* clone() const override;

  bool isActive(scalar_t time) const override;

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const ocs2::PreComputation& preComp) const override;

 private:
  EndEffectorVelocityConstraint(const EndEffectorVelocityConstraint& rhs) = default;

  int legNumber_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
};
}  // namespace switched_model
