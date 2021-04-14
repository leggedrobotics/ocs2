//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingCost : public CostFunctionBase {
 public:
  ~LoopshapingCost() override = default;

  LoopshapingCost(const LoopshapingCost& obj)
      : CostFunctionBase(), systemCost_(obj.systemCost_->clone()), loopshapingDefinition_(obj.loopshapingDefinition_) {}

  static std::unique_ptr<LoopshapingCost> create(const CostFunctionBase& systemCost,
                                                 std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr) override;
  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCost(scalar_t t, const vector_t& x) override;
  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) override;
  scalar_t costDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u) override;
  scalar_t finalCostDerivativeTime(scalar_t t, const vector_t& x) override;

 protected:
  LoopshapingCost(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : CostFunctionBase(), systemCost_(systemCost.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  std::unique_ptr<CostFunctionBase> systemCost_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  using CostFunctionBase::costDesiredTrajectoriesPtr_;
};

}  // namespace ocs2
