

#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>

namespace ocs2 {

class LoopshapingCostEliminatePattern final : public LoopshapingCost {
 public:
  using BASE = LoopshapingCost;

  LoopshapingCostEliminatePattern(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostEliminatePattern() override = default;

  LoopshapingCostEliminatePattern(const LoopshapingCostEliminatePattern& obj) = default;

  LoopshapingCostEliminatePattern* clone() const override { return new LoopshapingCostEliminatePattern(*this); };

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
