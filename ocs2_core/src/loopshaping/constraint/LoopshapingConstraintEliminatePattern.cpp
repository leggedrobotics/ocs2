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

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintEliminatePattern.h>

namespace ocs2 {

vector_array_t LoopshapingConstraintEliminatePattern::getInequalityConstraintDerivativesState() {
  vector_array_t dhdx;
  this->computeSystemInequalityConstraintDerivatives();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  dhdx.clear();
  if (systemConstraint_ && system_dhdx_.size() > 0) {
    const size_t SYSTEM_STATE_DIM = system_dhdx_.front().rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    dhdx.resize(system_dhdx_.size());
    for (size_t i = 0; i < system_dhdx_.size(); i++) {
      dhdx[i].resize(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
      dhdx[i].head(system_dhdx_[i].rows()) = system_dhdx_[i];
      dhdx[i].tail(s_filter.getNumStates()).noalias() = s_filter.getC().transpose() * system_dhdu_[i];
    }
  }
  return dhdx;
}

vector_array_t LoopshapingConstraintEliminatePattern::getInequalityConstraintDerivativesInput() {
  vector_array_t dhdu;
  this->computeSystemInequalityConstraintDerivatives();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (systemConstraint_) {
    dhdu.resize(system_dhdu_.size());
    for (size_t i = 0; i < system_dhdu_.size(); i++) {
      dhdu[i].noalias() = s_filter.getD().transpose() * system_dhdu_[i];
    }
  }
  return dhdu;
}

matrix_array_t LoopshapingConstraintEliminatePattern::getInequalityConstraintSecondDerivativesState() {
  matrix_array_t dhdxx;
  this->computeSystemInequalityConstraintDerivatives();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (systemConstraint_ && system_dhdxx_.size() > 0) {
    const size_t SYSTEM_STATE_DIM = system_dhdxx_.front().rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    dhdxx.resize(system_dhdxx_.size());
    for (size_t i = 0; i < system_dhdxx_.size(); i++) {
      dhdxx[i].resize(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
      dhdxx[i].topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_dhdxx_[i];
      dhdxx[i].topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = system_dhdux_[i].transpose() * s_filter.getC();
      dhdxx[i].bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM) =
          dhdxx[i].topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).transpose();
      dhdxx[i].bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).noalias() =
          s_filter.getC().transpose() * system_dhduu_[i] * s_filter.getC();
    }
  }
  return dhdxx;
}

matrix_array_t LoopshapingConstraintEliminatePattern::getInequalityConstraintSecondDerivativesInput() {
  matrix_array_t dhduu;
  this->computeSystemInequalityConstraintDerivatives();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (systemConstraint_) {
    dhduu.resize(system_dhduu_.size());
    for (size_t i = 0; i < system_dhduu_.size(); i++) {
      dhduu[i].noalias() = s_filter.getD().transpose() * system_dhduu_[i] * s_filter.getD();
    }
  }
  return dhduu;
}

matrix_array_t LoopshapingConstraintEliminatePattern::getInequalityConstraintDerivativesInputState() {
  matrix_array_t dhdux;
  this->computeSystemInequalityConstraintDerivatives();
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  if (systemConstraint_ && system_dhdux_.size()) {
    const size_t SYSTEM_STATE_DIM = system_dhdux_.front().cols();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    // assert(SYSTEM_INPUT_DIM == FILTER_INPUT_DIM);
    dhdux.resize(system_dhdux_.size());
    for (size_t i = 0; i < system_dhdux_.size(); i++) {
      dhdux[i].resize(FILTER_INPUT_DIM, FILTER_STATE_DIM + SYSTEM_STATE_DIM);
      dhdux[i].leftCols(SYSTEM_STATE_DIM).noalias() = s_filter.getD().transpose() * system_dhdux_[i];
      dhdux[i].rightCols(FILTER_STATE_DIM).noalias() = s_filter.getD().transpose() * system_dhduu_[i] * s_filter.getC();
    }
  }
  return dhdux;
}

void LoopshapingConstraintEliminatePattern::appendStateInputEqualityConstraintDerivativeState(matrix_t& C) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  // C.leftCols(SYSTEM_STATE_DIM), stays unaltered
  C.rightCols(FILTER_STATE_DIM).noalias() = D_system_ * s_filter.getC();
}

void LoopshapingConstraintEliminatePattern::appendStateInputEqualityConstraintDerivativeInput(matrix_t& D) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  // TODO(mspieler): This was the original implmentation, which looks wrong because it overwrites D.
  // D.leftCols(FILTER_INPUT_DIM).noalias() = D_system_ * s_filter.getD();
  D.noalias() = D_system_ * s_filter.getD();
}

}  // namespace ocs2
