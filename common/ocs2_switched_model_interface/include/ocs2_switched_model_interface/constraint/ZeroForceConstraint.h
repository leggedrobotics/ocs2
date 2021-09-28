#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h>

namespace switched_model {

/**
 * Implements the constraint that the contact forces of the foot must be zero for a foot in swing.
 */
class ZeroForceConstraint final : public ocs2::StateInputConstraint {
 public:
  explicit ZeroForceConstraint(int legNumber, const SwitchedModelModeScheduleManager& modeScheduleManager);

  ZeroForceConstraint* clone() const override;

  bool isActive(scalar_t time) const override;

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const ocs2::PreComputation& preComp) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const ocs2::PreComputation& preComp) const override;

 private:
  ZeroForceConstraint(const ZeroForceConstraint& rhs) = default;

  const int legStartIdx_;
  const int legNumber_;
  const SwitchedModelModeScheduleManager* modeScheduleManager_;
};

}  // namespace switched_model
