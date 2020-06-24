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

#include <ocs2_core/loopshaping/cost/LoopshapingCostInputPattern.h>

namespace ocs2 {

vector_t LoopshapingCostInputPattern::getCostDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  vector_t dLdx(SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdx.head(SYSTEM_STATE_DIM) = gamma * q_filter_ + (1.0 - gamma) * q_system_;
  dLdx.tail(FILTER_STATE_DIM).setZero();
  return dLdx;
}

matrix_t LoopshapingCostInputPattern::getCostSecondDerivativeState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();

  matrix_t dLdxx;
  dLdxx.setZero(SYSTEM_STATE_DIM + FILTER_STATE_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdxx.topLeftCorner(SYSTEM_STATE_DIM, SYSTEM_STATE_DIM) = gamma * Q_filter_ + (1.0 - gamma) * Q_system_;
  return dLdxx;
}

vector_t LoopshapingCostInputPattern::getCostDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const size_t FILTER_INPUT_DIM = r_filter_.rows();
  const size_t SYSTEM_INPUT_DIM = r_system_.rows();

  vector_t dLdu(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
  dLdu.head(SYSTEM_INPUT_DIM) = (1.0 - gamma) * r_system_;
  dLdu.tail(FILTER_INPUT_DIM) = gamma * r_filter_;
  return dLdu;
}

matrix_t LoopshapingCostInputPattern::getCostSecondDerivativeInput() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const size_t FILTER_INPUT_DIM = r_filter_.rows();
  const size_t SYSTEM_INPUT_DIM = r_system_.rows();

  matrix_t dLduu(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM, SYSTEM_INPUT_DIM + FILTER_INPUT_DIM);
  dLduu.topLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_INPUT_DIM) = (1.0 - gamma) * R_system_;
  dLduu.topRightCorner(SYSTEM_INPUT_DIM, FILTER_INPUT_DIM).setZero();
  dLduu.bottomLeftCorner(FILTER_INPUT_DIM, SYSTEM_INPUT_DIM).setZero();
  dLduu.bottomRightCorner(FILTER_INPUT_DIM, FILTER_INPUT_DIM) = gamma * R_filter_;
  return dLduu;
}

matrix_t LoopshapingCostInputPattern::getCostDerivativeInputState() {
  this->computeApproximation();
  const scalar_t gamma = loopshapingDefinition_->gamma_;
  const auto& s_filter = loopshapingDefinition_->getInputFilter();
  const size_t FILTER_STATE_DIM = s_filter.getNumStates();
  const size_t FILTER_INPUT_DIM = s_filter.getNumInputs();
  const size_t SYSTEM_STATE_DIM = q_system_.rows();
  const size_t SYSTEM_INPUT_DIM = r_system_.rows();

  matrix_t dLdux(SYSTEM_INPUT_DIM + FILTER_INPUT_DIM, SYSTEM_STATE_DIM + FILTER_STATE_DIM);
  dLdux.topLeftCorner(SYSTEM_INPUT_DIM, SYSTEM_STATE_DIM) = (1.0 - gamma) * P_system_;
  dLdux.topRightCorner(SYSTEM_INPUT_DIM, FILTER_STATE_DIM).setZero();
  dLdux.bottomLeftCorner(FILTER_INPUT_DIM, SYSTEM_STATE_DIM) = gamma * P_filter_;
  dLdux.bottomRightCorner(FILTER_INPUT_DIM, FILTER_STATE_DIM).setZero();
  return dLdux;
}

}  // namespace ocs2
