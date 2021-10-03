//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateInputCostCollection.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

/**
 * Loopshaping state-input cost collection base class
 */
class LoopshapingStateInputCost : public StateInputCostCollection {
 public:
  ~LoopshapingStateInputCost() override = default;

  scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const final;

 protected:
  /** Constructor */
  LoopshapingStateInputCost(const StateInputCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateInputCostCollection(systemCost), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  /** Copy constructor */
  LoopshapingStateInputCost(const LoopshapingStateInputCost& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
