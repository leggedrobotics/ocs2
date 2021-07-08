
#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingStateInputCost.h>

namespace ocs2 {

class LoopshapingCostEliminatePattern final : public LoopshapingStateInputCost {
 public:
  using BASE = LoopshapingStateInputCost;

  LoopshapingCostEliminatePattern(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostEliminatePattern() override = default;

  LoopshapingCostEliminatePattern(const LoopshapingCostEliminatePattern& obj) = default;

  LoopshapingCostEliminatePattern* clone() const override { return new LoopshapingCostEliminatePattern(*this); };

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
