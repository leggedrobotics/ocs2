
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

  explicit LoopshapingConstraintOutputPattern(size_t fullStateDim, size_t fullInputDim,
                                              std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(fullStateDim, fullInputDim, std::move(loopshapingDefinition)){};

  LoopshapingConstraintOutputPattern(size_t fullStateDim, size_t fullInputDim, const ConstraintBase& systemConstraint,
                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : BASE(fullStateDim, fullInputDim, systemConstraint, std::move(loopshapingDefinition)){};

  virtual ~LoopshapingConstraintOutputPattern() = default;

  LoopshapingConstraintOutputPattern(const LoopshapingConstraintOutputPattern& obj) = default;

  LoopshapingConstraintOutputPattern* clone() const override { return new LoopshapingConstraintOutputPattern(*this); };

  vector_array_t getInequalityConstraintDerivativesState() override;
  vector_array_t getInequalityConstraintDerivativesInput() override;
  matrix_array_t getInequalityConstraintSecondDerivativesState() override;
  matrix_array_t getInequalityConstraintSecondDerivativesInput() override;
  matrix_array_t getInequalityConstraintDerivativesInputState() override;

 protected:
  using BASE::loopshapingDefinition_;
  using BASE::systemConstraint_;

  using BASE::t_;
  using BASE::u_filter_;
  using BASE::u_system_;
  using BASE::x_filter_;
  using BASE::x_system_;

  using BASE::system_dhdu_;
  using BASE::system_dhduu_;
  using BASE::system_dhdux_;
  using BASE::system_dhdx_;
  using BASE::system_dhdxx_;
};

}  // namespace ocs2
