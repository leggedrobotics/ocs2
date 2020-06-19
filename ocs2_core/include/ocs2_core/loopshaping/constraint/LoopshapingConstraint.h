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

  LoopshapingConstraint(const LoopshapingConstraint& obj) : ConstraintBase(obj), loopshapingDefinition_(obj.loopshapingDefinition_) {
    if (obj.systemConstraint_) {
      systemConstraint_.reset(obj.systemConstraint_->clone());
    }
  }

  static std::unique_ptr<LoopshapingConstraint> create(size_t fullStateDim, size_t fullInputDim, const ConstraintBase& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  static std::unique_ptr<LoopshapingConstraint> create(size_t fullStateDim, size_t fullInputDim,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override;
  vector_t getStateInputEqualityConstraint() override;
  vector_t getStateEqualityConstraint() override;
  scalar_array_t getInequalityConstraint() override;
  vector_t getFinalStateEqualityConstraint() override;
  matrix_t getStateInputEqualityConstraintDerivativesState() override;
  matrix_t getStateInputEqualityConstraintDerivativesInput() override;
  matrix_t getStateEqualityConstraintDerivativesState() override;
  matrix_t getFinalStateEqualityConstraintDerivativesState() override;

 protected:
  LoopshapingConstraint(size_t fullStateDim, size_t fullInputDim, const ConstraintBase& systemConstraint,
                        std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : ConstraintBase(fullStateDim, fullInputDim),
        systemConstraint_(systemConstraint.clone()),
        loopshapingDefinition_(std::move(loopshapingDefinition)){};

  LoopshapingConstraint(size_t fullStateDim, size_t fullInputDim, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : ConstraintBase(fullStateDim, fullInputDim), systemConstraint_(nullptr), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  void computeSystemStateInputConstraintDerivatives();
  void computeSystemInequalityConstraintDerivatives();

 private:
  // Interface for derived classes to append additional constraints. Adding nothing by default
  virtual void appendStateInputEqualityConstraint(vector_t& e){};
  virtual void appendStateInputEqualityConstraintDerivativeState(matrix_t& C){};
  virtual void appendStateInputEqualityConstraintDerivativeInput(matrix_t& D){};

 protected:
  std::unique_ptr<ConstraintBase> systemConstraint_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;

  scalar_t t_;
  vector_t x_filter_;
  vector_t u_filter_;
  vector_t x_system_;
  vector_t u_system_;

  matrix_t C_system_;
  matrix_t D_system_;

  vector_array_t system_dhdx_;
  vector_array_t system_dhdu_;
  matrix_array_t system_dhdxx_;
  matrix_array_t system_dhduu_;
  matrix_array_t system_dhdux_;

 private:
  bool systemStateInputConstraintApproximationValid_ = false;
  bool systemInequalityConstraintApproximationValid_ = false;
};

}  // namespace ocs2
