#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_legged_robot_example/common/definitions.h>

namespace ocs2 {
namespace legged_robot {

class ZeroForceConstraint final : public ocs2::StateInputConstraint {
 public:
  using BASE = ocs2::StateInputConstraint;

  explicit ZeroForceConstraint(int legNumber);

  ZeroForceConstraint* clone() const override;

  size_t getNumConstraints(scalar_t time) const override;

  vector_t getValue(scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input) const override;

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const ocs2::vector_t& state,
                                                           const vector_t& input) const override;

 private:
  int contactPointNumber_;
};

}  // namespace legged_robot
}  // namespace ocs2
