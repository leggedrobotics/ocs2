
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

  LoopshapingConstraintEliminatePattern(size_t fullStateDim, size_t fullInputDim,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(fullStateDim, fullInputDim, std::move(loopshapingDefinition)){};

  LoopshapingConstraintEliminatePattern(size_t fullStateDim, size_t fullInputDim, const ConstraintBase& systemConstraint,
                                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(fullStateDim, fullInputDim, systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintEliminatePattern() = default;

  LoopshapingConstraintEliminatePattern(const LoopshapingConstraintEliminatePattern& obj) = default;

  LoopshapingConstraintEliminatePattern* clone() const override { return new LoopshapingConstraintEliminatePattern(*this); };

  vector_array_t getInequalityConstraintDerivativesState() override;
  vector_array_t getInequalityConstraintDerivativesInput() override;
  matrix_array_t getInequalityConstraintSecondDerivativesState() override;
  matrix_array_t getInequalityConstraintSecondDerivativesInput() override;
  matrix_array_t getInequalityConstraintDerivativesInputState() override;
  void appendStateInputEqualityConstraintDerivativeState(matrix_t& C) override;
  void appendStateInputEqualityConstraintDerivativeInput(matrix_t& D) override;

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemConstraint_;

  using BASE::t_;
  using BASE::u_filter_;
  using BASE::u_system_;
  using BASE::x_filter_;
  using BASE::x_system_;

  using BASE::C_system_;
  using BASE::D_system_;

  using BASE::system_dhdu_;
  using BASE::system_dhduu_;
  using BASE::system_dhdux_;
  using BASE::system_dhdx_;
  using BASE::system_dhdxx_;
};

}  // namespace ocs2
