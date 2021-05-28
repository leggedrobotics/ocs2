//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingStateConstraint final : public StateConstraintCollection {
 public:
  LoopshapingStateConstraint(const StateConstraintCollection& systemConstraint,
                             std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateConstraintCollection(systemConstraint), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  ~LoopshapingStateConstraint() override = default;

  LoopshapingStateConstraint* clone() const override { return new LoopshapingStateConstraint(*this); }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;

 private:
  LoopshapingStateConstraint(const LoopshapingStateConstraint& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
