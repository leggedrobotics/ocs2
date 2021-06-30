
#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingStateInputCost.h>

namespace ocs2 {

class LoopshapingCostInputPattern final : public LoopshapingStateInputCost {
 public:
  using BASE = LoopshapingStateInputCost;

  LoopshapingCostInputPattern(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostInputPattern() override = default;

  LoopshapingCostInputPattern(const LoopshapingCostInputPattern& obj) = default;

  LoopshapingCostInputPattern* clone() const override { return new LoopshapingCostInputPattern(*this); };

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const CostDesiredTrajectories& desiredTrajectory,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
