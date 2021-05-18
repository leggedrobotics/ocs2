//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingStateInputConstraint : public StateInputConstraint {
 public:
  ~LoopshapingStateInputConstraint() override = default;

  LoopshapingStateInputConstraint* clone() const override { new LoopshapingStateInputConstraint(*this); }

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation* preCompPtr) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation* preCompPtr) const override;
  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation* preCompPtr) const override;

 protected:
  LoopshapingStateInputConstraint(const StateInputConstraint& systemConstraint,
                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateInputConstraint(systemConstraint.getOrder()),
        systemConstraint_(systemConstraint.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)){};

  LoopshapingStateInputConstraint(const LoopshapingStateInputConstraint& other)
      : StateInputConstraint(other), loopshapingDefinition_(other.loopshapingDefinition_) {
    systemConstraint_.reset(other.systemConstraint_->clone());
  }

  std::unique_ptr<ConstraintBase> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
