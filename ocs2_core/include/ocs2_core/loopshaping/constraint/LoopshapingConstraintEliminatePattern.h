
#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingStateInputConstraint.h>

namespace ocs2 {

/*
 * FULL_STATE_DIM = SYSTEM_STATE_DIM + FILTER_STATE_DIM
 * FULL_INPUT_DIM = FILTER_INPUT_DIM
 */
class LoopshapingConstraintEliminatePattern final : public LoopshapingStateInputConstraint {
 public:
  using BASE = LoopshapingStateInputConstraint;

  LoopshapingConstraintEliminatePattern(const StateInputConstraintCollection& systemConstraint,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(systemConstraint, std::move(loopshapingDefinition)){};

  ~LoopshapingConstraintEliminatePattern() override = default;

  LoopshapingConstraintEliminatePattern(const LoopshapingConstraintEliminatePattern& obj) = default;

  LoopshapingConstraintEliminatePattern* clone() const override { return new LoopshapingConstraintEliminatePattern(*this); };

  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                           const PreComputation& preComp) const override;

  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

 protected:
  using BASE::loopshapingDefinition_;
};

}  // namespace ocs2
