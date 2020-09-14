
#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>

namespace ocs2 {

class LoopshapingCostInputPattern final : public LoopshapingCost {
 public:
  using BASE = LoopshapingCost;

  LoopshapingCostInputPattern(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostInputPattern() override = default;

  LoopshapingCostInputPattern(const LoopshapingCostInputPattern& obj) = default;

  LoopshapingCostInputPattern* clone() const override { return new LoopshapingCostInputPattern(*this); };

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) override;

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemCost_;
};

}  // namespace ocs2
