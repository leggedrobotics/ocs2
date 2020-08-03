
#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>

namespace ocs2 {

class LoopshapingCostOutputPattern final : public LoopshapingCost {
 public:
  using BASE = LoopshapingCost;

  LoopshapingCostOutputPattern(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostOutputPattern() override = default;

  LoopshapingCostOutputPattern(const LoopshapingCostOutputPattern& obj) = default;

  LoopshapingCostOutputPattern* clone() const override { return new LoopshapingCostOutputPattern(*this); };

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemCost_;
};

}  // namespace ocs2
