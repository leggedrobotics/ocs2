/******************************************************************************
Copyright (c) 2020, Ruben Grandia. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraint.h>

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintInputPattern.h>
#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintOutputPattern.h>

namespace ocs2 {

std::unique_ptr<LoopshapingConstraint> LoopshapingConstraint::create(size_t fullStateDim, size_t fullInputDim,
                                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintOutputPattern(fullStateDim, fullInputDim, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintInputPattern(fullStateDim, fullInputDim, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintEliminatePattern(fullStateDim, fullInputDim, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingConstraint::create] invalid loopshaping type");
  }
}

std::unique_ptr<LoopshapingConstraint> LoopshapingConstraint::create(size_t fullStateDim, size_t fullInputDim,
                                                                     const ConstraintBase& systemConstraint,
                                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  switch (loopshapingDefinition->getType()) {
    case LoopshapingType::outputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintOutputPattern(fullStateDim, fullInputDim, systemConstraint, std::move(loopshapingDefinition)));
    case LoopshapingType::inputpattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintInputPattern(fullStateDim, fullInputDim, systemConstraint, std::move(loopshapingDefinition)));
    case LoopshapingType::eliminatepattern:
      return std::unique_ptr<LoopshapingConstraint>(
          new LoopshapingConstraintEliminatePattern(fullStateDim, fullInputDim, systemConstraint, std::move(loopshapingDefinition)));
    default:
      throw std::runtime_error("[LoopshapingConstraint::create] invalid loopshaping type");
  }
}

void LoopshapingConstraint::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  systemStateInputConstraintApproximationValid_ = false;
  systemInequalityConstraintApproximationValid_ = false;

  ConstraintBase::setCurrentStateAndControl(t, x, u);

  t_ = t;
  x_system_ = loopshapingDefinition_->getSystemState(x);
  u_system_ = loopshapingDefinition_->getSystemInput(x, u);
  x_filter_ = loopshapingDefinition_->getFilterState(x);
  u_filter_ = loopshapingDefinition_->getFilteredInput(x, u);

  if (systemConstraint_) {
    systemConstraint_->setCurrentStateAndControl(t, x_system_, u_system_);
  }
}

vector_t LoopshapingConstraint::getStateInputEqualityConstraint() {
  vector_t e(0);
  if (systemConstraint_) {
    e = systemConstraint_->getStateInputEqualityConstraint();
  }
  appendStateInputEqualityConstraint(e);
  return e;
}

vector_t LoopshapingConstraint::getStateEqualityConstraint() {
  if (systemConstraint_) {
    return systemConstraint_->getStateEqualityConstraint();
  } else {
    return vector_t(0);
  }
}

scalar_array_t LoopshapingConstraint::getInequalityConstraint() {
  if (systemConstraint_) {
    return systemConstraint_->getInequalityConstraint();
  } else {
    return scalar_array_t(0);
  }
}

vector_t LoopshapingConstraint::getFinalStateEqualityConstraint() {
  if (systemConstraint_) {
    return systemConstraint_->getFinalStateEqualityConstraint();
  } else {
    return vector_t(0);
  }
}

matrix_t LoopshapingConstraint::getStateInputEqualityConstraintDerivativesState() {
  computeSystemStateInputConstraintDerivatives();  // updates C_system_
  matrix_t C(0, ConstraintBase::stateDim_);
  if (systemConstraint_) {
    C_system_ = systemConstraint_->getStateInputEqualityConstraintDerivativesState();
    C.resize(C_system_.rows(), ConstraintBase::stateDim_);
    C.leftCols(C_system_.cols()) = C_system_;
    C.rightCols(ConstraintBase::stateDim_ - C_system_.cols()).setZero();
  }
  appendStateInputEqualityConstraintDerivativeState(C);
  return C;
}

matrix_t LoopshapingConstraint::getStateInputEqualityConstraintDerivativesInput() {
  computeSystemStateInputConstraintDerivatives();  // updates D_system_
  matrix_t D(0, ConstraintBase::inputDim_);
  if (systemConstraint_) {
    D.resize(D_system_.rows(), ConstraintBase::inputDim_);
    D.leftCols(D_system_.cols()) = D_system_;
    D.rightCols(ConstraintBase::inputDim_ - D_system_.cols()).setZero();
  }
  appendStateInputEqualityConstraintDerivativeInput(D);
  return D;
}

matrix_t LoopshapingConstraint::getStateEqualityConstraintDerivativesState() {
  matrix_t F(0, ConstraintBase::stateDim_);
  if (systemConstraint_) {
    matrix_t F_system = systemConstraint_->getStateEqualityConstraintDerivativesState();
    F.resize(F_system.rows(), ConstraintBase::stateDim_);
    F.leftCols(F_system.cols()) = F_system;
    F.rightCols(ConstraintBase::stateDim_ - F_system.cols()).setZero();
  }
  return F;
}

matrix_t LoopshapingConstraint::getFinalStateEqualityConstraintDerivativesState() {
  matrix_t F_f(0, ConstraintBase::stateDim_);
  if (systemConstraint_) {
    matrix_t F_f_system = systemConstraint_->getFinalStateEqualityConstraintDerivativesState();
    F_f.resize(F_f_system.rows(), ConstraintBase::stateDim_);
    F_f.leftCols(F_f_system.cols()) = F_f_system;
    F_f.rightCols(ConstraintBase::stateDim_ - F_f_system.cols()).setZero();
  }
  return F_f;
}

void LoopshapingConstraint::computeSystemStateInputConstraintDerivatives() {
  if (!systemStateInputConstraintApproximationValid_) {
    if (systemConstraint_) {
      C_system_ = systemConstraint_->getStateInputEqualityConstraintDerivativesState();
      D_system_ = systemConstraint_->getStateInputEqualityConstraintDerivativesInput();
    }
    systemStateInputConstraintApproximationValid_ = true;
  }
}

void LoopshapingConstraint::computeSystemInequalityConstraintDerivatives() {
  if (!systemInequalityConstraintApproximationValid_) {
    if (systemConstraint_) {
      system_dhdx_ = systemConstraint_->getInequalityConstraintDerivativesState();
      system_dhdu_ = systemConstraint_->getInequalityConstraintDerivativesInput();
      system_dhdxx_ = systemConstraint_->getInequalityConstraintSecondDerivativesState();
      system_dhduu_ = systemConstraint_->getInequalityConstraintSecondDerivativesInput();
      system_dhdux_ = systemConstraint_->getInequalityConstraintDerivativesInputState();
      systemInequalityConstraintApproximationValid_ = true;
    }
  }
}

}  // namespace ocs2
