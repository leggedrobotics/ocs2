//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/LoopshapingPreComputation.h>

namespace ocs2 {

class LoopshapingCost final : public CostFunctionBase {
 public:
  using Base = CostFunctionBase;

  LoopshapingCost(const CostFunctionBase& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition,
                  std::shared_ptr<LoopshapingPreComputation> preComputation);

  ~LoopshapingCost() override = default;

  CostFunctionBase* clone(std::shared_ptr<PreComputation> preCompPtr) const override;

 protected:
  LoopshapingCost(const LoopshapingCost& other, std::shared_ptr<PreComputation> preCompPtr);
};

}  // namespace ocs2
