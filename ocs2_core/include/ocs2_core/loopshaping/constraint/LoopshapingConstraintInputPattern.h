
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraint.h>

namespace ocs2 {

/*
 * FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM
 * FULL_INPUT_DIM = SYSTEM_INPUT_DIM + FILTER_INPUT_DIM
 */
class LoopshapingConstraintInputPattern final : public LoopshapingConstraint {
 public:
  using BASE = LoopshapingConstraint;

  explicit LoopshapingConstraintInputPattern(std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(std::move(loopshapingDefinition)){};

  LoopshapingConstraintInputPattern(const ConstraintBase& systemConstraint, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  ~LoopshapingConstraintInputPattern() override = default;

  LoopshapingConstraintInputPattern(const LoopshapingConstraintInputPattern& obj) = default;

  LoopshapingConstraintInputPattern* clone() const override { return new LoopshapingConstraintInputPattern(*this); };

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
