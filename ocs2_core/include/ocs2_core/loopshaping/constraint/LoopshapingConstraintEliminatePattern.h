
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraint.h>

namespace ocs2 {

/*
 * FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM
 * FULL_INPUT_DIM = FILTER_INPUT_DIM
 */
class LoopshapingConstraintEliminatePattern final : public LoopshapingConstraint {
 public:
  using BASE = LoopshapingConstraint;

  explicit LoopshapingConstraintEliminatePattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintEliminatePattern(const ConstraintBase& systemConstraint,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintEliminatePattern() = default;

  LoopshapingConstraintEliminatePattern(const LoopshapingConstraintEliminatePattern& obj) = default;

  LoopshapingConstraintEliminatePattern* clone() const override { return new LoopshapingConstraintEliminatePattern(*this); };

  vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) override;
  VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                    const vector_t& u) override;
  VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                  const vector_t& u) override;

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemConstraint_;
};

}  // namespace ocs2
