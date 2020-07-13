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

#include <ocs2_core/loopshaping/cost/LoopshapingCostOutputPattern.h>

namespace ocs2 {

vector_t LoopshapingCostOutputPattern::getCostDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  vector_t dLdx(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdx.head(SYSTEM_STATE_DIM) = gamma * q_filter_ + (1.0 - gamma) * q_system_;
  dLdx.tail(FILTER_STATE_DIM) = gamma * r_filter.getC().transpose() * r_filter_;
  return dLdx;
}

matrix_t LoopshapingCostOutputPattern::getCostSecondDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  matrix_t dLdxx(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdxx.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = gamma * Q_filter_ + (1.0 - gamma) * Q_system_;
  dLdxx.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM) = gamma * P_filter_.transpose() * r_filter.getC();
  dLdxx.bottomLeftCorner(FILTER_STATE_DIM, SYSTEM_STATE_DIM) = dLdxx.topRightCorner(SYSTEM_STATE_DIM, FILTER_STATE_DIM).transpose();
  dLdxx.bottomRightCorner(FILTER_STATE_DIM, FILTER_STATE_DIM) = gamma * r_filter.getC().transpose() * R_filter_ * r_filter.getC();
  return dLdxx;
}

vector_t LoopshapingCostOutputPattern::getCostDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& r_filter = loopshapingDefinition_->getInputFilter();

  return gamma * r_filter.getD().transpose() * r_filter_ + (1.0 - gamma) * r_system_;
}

matrix_t LoopshapingCostOutputPattern::getCostSecondDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& r_filter = loopshapingDefinition_->getInputFilter();

  return gamma * r_filter.getD().transpose() * R_filter_ * r_filter.getD() + (1.0 - gamma) * R_system_;
}

matrix_t LoopshapingCostOutputPattern::getCostDerivativeInputState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& r_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = r_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = P_system_.cols();
  const size_t SYSTEM_INPUT_DIM = P_system_.rows();

  matrix_t dLdux(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdux.leftCols(SYSTEM_STATE_DIM) = gamma * r_filter.getD().transpose() * P_filter_ + (1.0 - gamma) * P_system_;
  dLdux.rightCols(FILTER_STATE_DIM) = gamma * r_filter.getD().transpose() * R_filter_ * r_filter.getC();
  return dLdux;
}

}  // namespace ocs2