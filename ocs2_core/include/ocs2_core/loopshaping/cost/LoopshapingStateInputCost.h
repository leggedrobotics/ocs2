//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

/**
 * Loopshaping state-input cost decorator base class
 */
class LoopshapingStateInputCost : public StateInputCost {
 public:
  ~LoopshapingStateInputCost() override = default;

  scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const CostDesiredTrajectories& desiredTrajectory,
                    const PreComputation& preComp) const final;

 protected:
  /** Constructor */
  LoopshapingStateInputCost(const StateInputCost& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateInputCost(), systemCost_(systemCost.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  /** Copy constructor */
  LoopshapingStateInputCost(const LoopshapingStateInputCost& other)
      : StateInputCost(), systemCost_(other.systemCost_->clone()), loopshapingDefinition_(other.loopshapingDefinition_) {}

  std::unique_ptr<StateInputCost> systemCost_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
