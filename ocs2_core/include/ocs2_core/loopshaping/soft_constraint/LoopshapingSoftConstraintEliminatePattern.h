
#pragma once

#include <ocs2_core/loopshaping/soft_constraint/LoopshapingStateInputSoftConstraint.h>

namespace ocs2 {

class LoopshapingSoftConstraintEliminatePattern final : public LoopshapingStateInputSoftConstraint {
 public:
  using BASE = LoopshapingStateInputSoftConstraint;

  LoopshapingSoftConstraintEliminatePattern(const StateInputCostCollection& systemCost,
                                            std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingSoftConstraintEliminatePattern() override = default;

  LoopshapingSoftConstraintEliminatePattern(const LoopshapingSoftConstraintEliminatePattern& obj) = default;

  LoopshapingSoftConstraintEliminatePattern* clone() const override { return new LoopshapingSoftConstraintEliminatePattern(*this); };

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const CostDesiredTrajectories& desiredTrajectory,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
