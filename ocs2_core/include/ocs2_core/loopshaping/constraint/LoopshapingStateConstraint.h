//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingStateConstraint final : public StateConstraint {
 public:
  LoopshapingStateConstraint(const StateConstraint& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateConstraint(systemConstraint.getOrder()),
        systemConstraint_(systemConstraint.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)){};

  ~LoopshapingStateConstraint() override = default;

  LoopshapingStateConstraint* clone() const override { return new LoopshapingStateConstraint(*this); }

  size_t getNumConstraints(scalar_t time) const override { return systemConstraint_->getNumConstraints(time); }

  vector_t getValue(scalar_t time, const vector_t& state, const PreComputation* preCompPtr) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation* preCompPtr) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation* preCompPtr) const override;

 private:
  LoopshapingStateConstraint(const LoopshapingStateConstraint& other)
      : StateConstraint(other), loopshapingDefinition_(other.loopshapingDefinition_) {
    systemConstraint_.reset(other.systemConstraint_->clone());
  }

  std::unique_ptr<StateConstraint> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
