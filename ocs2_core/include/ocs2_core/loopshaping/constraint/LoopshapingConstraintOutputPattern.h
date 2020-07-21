
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraint.h>

namespace ocs2 {

/*
 * FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM
 * FULL_INPUT_DIM = SYSTEM_INPUT_DIM
 */
class LoopshapingConstraintOutputPattern final : public LoopshapingConstraint {
 public:
  using BASE = LoopshapingConstraint;

  explicit LoopshapingConstraintOutputPattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintOutputPattern(const ConstraintBase& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintOutputPattern() = default;

  LoopshapingConstraintOutputPattern(const LoopshapingConstraintOutputPattern& obj) = default;

  LoopshapingConstraintOutputPattern* clone() const override { return new LoopshapingConstraintOutputPattern(*this); };

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
