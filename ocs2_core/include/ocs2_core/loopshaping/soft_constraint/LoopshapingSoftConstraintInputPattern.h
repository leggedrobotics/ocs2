
#pragma once

#include <ocs2_core/loopshaping/soft_constraint/LoopshapingStateInputSoftConstraint.h>

namespace ocs2 {

class LoopshapingSoftConstraintInputPattern final : public LoopshapingStateInputSoftConstraint {
 public:
  using BASE = LoopshapingStateInputSoftConstraint;

  LoopshapingSoftConstraintInputPattern(const StateInputCostCollection& systemCost,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingSoftConstraintInputPattern() override = default;

  LoopshapingSoftConstraintInputPattern(const LoopshapingSoftConstraintInputPattern& obj) = default;

  LoopshapingSoftConstraintInputPattern* clone() const override { return new LoopshapingSoftConstraintInputPattern(*this); };

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const CostDesiredTrajectories& desiredTrajectory,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
