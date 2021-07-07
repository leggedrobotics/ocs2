//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCostCollection.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

/**
 * Loopshaping state-only cost collection class
 */
class LoopshapingStateCost final : public StateCostCollection {
 public:
  LoopshapingStateCost(const StateCostCollection& systemCost, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateCostCollection(systemCost), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  ~LoopshapingStateCost() override = default;

  LoopshapingStateCost* clone() const override { return new LoopshapingStateCost(*this); }

  scalar_t getValue(scalar_t t, const vector_t& x, const TargetTrajectories& targetTrajectories,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                 const TargetTrajectories& targetTrajectories,
                                                                 const PreComputation& preComp) const override;

 private:
  LoopshapingStateCost(const LoopshapingStateCost& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
