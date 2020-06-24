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

#include <ocs2_core/loopshaping/cost/LoopshapingCostEliminatePattern.h>

namespace ocs2 {

vector_t LoopshapingCostEliminatePattern::getCostDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  vector_t dLdx(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdx.head(SYSTEM_STATE_DIM).noalias() = gamma * q_filter_ + (1.0 - gamma) * q_system_;
  dLdx.tail(FILTER_STATE_DIM).noalias() = (1.0 - gamma) * s_filter.getC().transpose() * r_system_;
  return dLdx;
}

matrix_t LoopshapingCostEliminatePattern::getCostSecondDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  matrix_t dLdxx(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdxx.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM).noalias() = gamma * Q_filter_ + (1.0 - gamma) * Q_system_;
  dLdxx.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).noalias() = (1.0 - gamma) * P_system_.transpose() * s_filter.getC();
  dLdxx.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM) = dLdxx.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).transpose();
  dLdxx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM).noalias() =
      (1.0 - gamma) * s_filter.getC().transpose() * R_system_ * s_filter.getC();
  return dLdxx;
}

vector_t LoopshapingCostEliminatePattern::getCostDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();

  return gamma * r_filter_ + (1.0 - gamma) * s_filter.getD().transpose() * r_system_;
}

matrix_t LoopshapingCostEliminatePattern::getCostSecondDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();

  matrix_t dLduu = gamma * R_filter_;
  dLduu.noalias() += (1.0 - gamma) * s_filter.getD().transpose() * R_system_ * s_filter.getD();
  return dLduu;
}

matrix_t LoopshapingCostEliminatePattern::getCostDerivativeInputState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const size_t SYSTEM_STATE_DIM = P_filter_.cols();

  matrix_t dLdux(FILTER_INPUT_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdux.leftCols(SYSTEM_STATE_DIM) = gamma * P_filter_;
  dLdux.leftCols(SYSTEM_STATE_DIM).noalias() += (1.0 - gamma) * s_filter.getD().transpose() * P_system_;
  dLdux.rightCols(FILTER_STATE_DIM).noalias() = (1.0 - gamma) * s_filter.getD().transpose() * R_system_ * s_filter.getC();
  return dLdux;
}

}  // namespace ocs2
