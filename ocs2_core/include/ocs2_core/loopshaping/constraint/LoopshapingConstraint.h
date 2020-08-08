//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingConstraint : public ConstraintBase {
 public:
  ~LoopshapingConstraint() override = default;

  static std::unique_ptr<LoopshapingConstraint> create(const ConstraintBase& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  static std::unique_ptr<LoopshapingConstraint> create(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  vector_t stateEqualityConstraint(scalar_t t, const vector_t& x) final;
  vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) final;
  vector_t finalStateEqualityConstraint(scalar_t t, const vector_t& x) final;

  VectorFunctionLinearApproximation stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) final;
  VectorFunctionLinearApproximation finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) final;

 protected:
  LoopshapingConstraint(const ConstraintBase& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : systemConstraint_(systemConstraint.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  explicit LoopshapingConstraint(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : systemConstraint_(new ConstraintBase()), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  LoopshapingConstraint(const LoopshapingConstraint& obj) : ConstraintBase(obj), loopshapingDefinition_(obj.loopshapingDefinition_) {
    systemConstraint_.reset(obj.systemConstraint_->clone());
  }

 protected:
  std::unique_ptr<ConstraintBase> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
