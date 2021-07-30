#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

namespace switched_model {

class ZeroForceConstraint final : public ocs2::StateInputConstraint {
 public:
  explicit ZeroForceConstraint(int legNumber);

  ZeroForceConstraint* clone() const override;

  size_t getNumConstraints(scalar_t time) const override;
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input) const override;

 private:
  ZeroForceConstraint(const ZeroForceConstraint& rhs) = default;

  const int legStartIdx_;
};

}  // namespace switched_model
