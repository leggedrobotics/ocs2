

#pragma once

#include <ocs2_core/loopshaping/cost/LoopshapingCost.h>

namespace ocs2 {

/*
 * FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM
 * FULL_INPUT_DIM = FILTER_INPUT_DIM
 */
class LoopshapingCostEliminatePattern final : public LoopshapingCost {
 public:
  using BASE = LoopshapingCost;

  LoopshapingCostEliminatePattern(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemCost, std::move(loopshapingDefinition)) {}

  ~LoopshapingCostEliminatePattern() override = default;

  LoopshapingCostEliminatePattern(const LoopshapingCostEliminatePattern& obj) = default;

  LoopshapingCostEliminatePattern* clone() const override { return new LoopshapingCostEliminatePattern(*this); };

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
