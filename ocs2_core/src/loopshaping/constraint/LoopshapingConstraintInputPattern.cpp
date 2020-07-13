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

#include <ocs2_core/loopshaping/constraint/LoopshapingConstraintInputPattern.h>

namespace ocs2 {

vector_array_t LoopshapingConstraintInputPattern::getInequalityConstraintDerivativesState() {
  vector_array_t dhdx;
  this->computeSystemInequalityConstraintDerivatives();
  dhdx.clear();
  if (systemConstraint_ && system_dhdx_.size() > 0) {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = system_dhdx_.front().rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    dhdx.resize(system_dhdx_.size());
    for (size_t i = 0; i < system_dhdx_.size(); i++) {
      dhdx[i].resize(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
      dhdx[i].head(SYSTEM_STATE_DIM) = system_dhdx_[i];
      dhdx[i].tail(FILTER_STATE_DIM).setZero();
    }
  }
  return dhdx;
}

vector_array_t LoopshapingConstraintInputPattern::getInequalityConstraintDerivativesInput() {
  vector_array_t dhdu;
  this->computeSystemInequalityConstraintDerivatives();
  if (systemConstraint_ && system_dhdu_.size() > 0) {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_INPUT_DIM = system_dhdu_.front().rows();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    dhdu.resize(system_dhdu_.size());
    for (size_t i = 0; i < system_dhdu_.size(); i++) {
      dhdu[i].resize(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
      dhdu[i].head(SYSTEM_INPUT_DIM) = system_dhdu_[i];
      dhdu[i].tail(FILTER_INPUT_DIM).setZero();
    }
  }
  return dhdu;
}

matrix_array_t LoopshapingConstraintInputPattern::getInequalityConstraintSecondDerivativesState() {
  matrix_array_t dhdxx;
  this->computeSystemInequalityConstraintDerivatives();
  if (systemConstraint_ && system_dhdxx_.size() > 0) {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = system_dhdxx_.front().rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    dhdxx.resize(system_dhdxx_.size());
    for (size_t i = 0; i < system_dhdxx_.size(); i++) {
      dhdxx[i].resize(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
      dhdxx[i].topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = system_dhdxx_[i];
      dhdxx[i].topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).setZero();
      dhdxx[i].bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM).setZero();
      dhdxx[i].bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).setZero();
    }
  }
  return dhdxx;
}

matrix_array_t LoopshapingConstraintInputPattern::getInequalityConstraintSecondDerivativesInput() {
  matrix_array_t dhduu;
  this->computeSystemInequalityConstraintDerivatives();
  if (systemConstraint_ && system_dhduu_.size() > 0) {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_INPUT_DIM = system_dhduu_.front().rows();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    dhduu.resize(system_dhduu_.size());
    for (size_t i = 0; i < system_dhduu_.size(); i++) {
      dhduu[i].resize(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM, SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
      dhduu[i].topLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = system_dhduu_[i];
      dhduu[i].topRightCorner(SYSTEM_INPUT_DIM, FILTER_INPUT_DIM).setZero();
      dhduu[i].bottomLeftCorner(FILTER_INPUT_DIM, SYSTEM_INPUT_DIM).setZero();
      dhduu[i].bottomRightCorner(FILTER_INPUT_DIM, FILTER_INPUT_DIM).setZero();
    }
  }
  return dhduu;
}

matrix_array_t LoopshapingConstraintInputPattern::getInequalityConstraintDerivativesInputState() {
  matrix_array_t dhdux;
  this->computeSystemInequalityConstraintDerivatives();
  if (systemConstraint_ && system_dhdux_.size() > 0) {
    const auto& s_filter = loopshapingDefinition_->getInputFilter();
    const size_t SYSTEM_STATE_DIM = system_dhdux_.front().cols();
    const size_t SYSTEM_INPUT_DIM = system_dhdux_.front().rows();
    const size_t FILTER_STATE_DIM = s_filter.getNumStates();
    const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
    dhdux.resize(system_dhdux_.size());
    for (size_t i = 0; i < system_dhdux_.size(); i++) {
      dhdux[i].resize(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
      dhdux[i].topLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM) = system_dhdux_[i];
      dhdux[i].topRightCorner(SYSTEM_INPUT_DIM, FILTER_STATE_DIM).setZero();
      dhdux[i].bottomLeftCorner(FILTER_INPUT_DIM, SYSTEM_STATE_DIM).setZero();
      dhdux[i].bottomRightCorner(FILTER_INPUT_DIM, FILTER_STATE_DIM).setZero();
    }
  }
  return dhdux;
}

void LoopshapingConstraintInputPattern::appendStateInputEqualityConstraint(vector_t& e) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t SYSTEM_INPUT_DIM = u_system_.rows();
  e.conservativeResize(e.rows() + SYSTEM_INPUT_DIM);
  e.tail(SYSTEM_INPUT_DIM) = s_filter.getC() * x_filter_ + s_filter.getD() * u_filter_ - u_system_;
}

void LoopshapingConstraintInputPattern::appendStateInputEqualityConstraintDerivativeState(matrix_t& C) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = x_system_.rows();
  const size_t SYSTEM_INPUT_DIM = u_system_.rows();
  C.conservativeResize(C.rows() + SYSTEM_INPUT_DIM, C.cols());
  C.bottomLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM).setZero();
  C.bottomRightCorner(SYSTEM_INPUT_DIM, FILTER_STATE_DIM) = s_filter.getC();
}

void LoopshapingConstraintInputPattern::appendStateInputEqualityConstraintDerivativeInput(matrix_t& D) {
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const size_t SYSTEM_INPUT_DIM = u_system_.rows();
  D.conservativeResize(D.rows() + SYSTEM_INPUT_DIM, D.cols());
  D.bottomLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = -matrix_t::Identity(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM);
  D.bottomRightCorner(SYSTEM_INPUT_DIM, FILTER_INPUT_DIM) = s_filter.getD();
}

}  // namespace ocs2
