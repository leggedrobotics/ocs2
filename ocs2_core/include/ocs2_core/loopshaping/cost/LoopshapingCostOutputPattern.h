
#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingStateInputCost.h>

namespace ocs2 {

class LoopshapingCostOutputPattern final : public LoopshapingStateInputCost {
 public:
  using BASE = LoopshapingStateInputCost;

  LoopshapingCostOutputPattern(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostOutputPattern() override = default;

  LoopshapingCostOutputPattern(const LoopshapingCostOutputPattern& obj) = default;

  LoopshapingCostOutputPattern* clone() const override { return new LoopshapingCostOutputPattern(*this); };

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
