
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

  vector_t getCostDerivativeState() override;

  matrix_t getCostSecondDerivativeState() override;

  vector_t getCostDerivativeInput() override;

  matrix_t getCostSecondDerivativeInput() override;

  matrix_t getCostDerivativeInputState() override;

 protected:
  using BASE::c_system_;
  using BASE::loopshapingDefinition_;
  using BASE::P_system_;
  using BASE::Q_system_;
  using BASE::q_system_;
  using BASE::R_system_;
  using BASE::r_system_;

  using BASE::c_filter_;
  using BASE::P_filter_;
  using BASE::Q_filter_;
  using BASE::q_filter_;
  using BASE::R_filter_;
  using BASE::r_filter_;
};

}  // namespace ocs2
