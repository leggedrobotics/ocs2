/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/constraint/PenaltyBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCost(const scalar_array_t& h, scalar_t& penalty) const {
  penalty = 0;
  for (const scalar_t& hi : h) {
    penalty += getPenaltyFunctionValue(hi);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                                                      state_vector_t& penaltyDerivativeState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdx[i];
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu,
                                                                      input_vector_t& penaltyDerivativeInput) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeInput.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdu[i];
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                                                            const dynamic_matrix_array_t& ddhdxdx,
                                                                            state_matrix_t& penaltySecondDerivativeState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != ddhdxdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltySecondDerivativeState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdxdx[i];
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdx[i] * dhdx[i].transpose();
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu,
                                                                            const dynamic_matrix_array_t& ddhdudu,
                                                                            input_matrix_t& penaltySecondDerivativeInput) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size() || numInequalityConstraints != ddhdudu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltySecondDerivativeInput.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudu[i];
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdu[i].transpose();
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getPenaltyCostDerivativeInputState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                                                           const dynamic_vector_array_t& dhdu,
                                                                           const dynamic_matrix_array_t& ddhdudx,
                                                                           input_state_matrix_t& penaltyDerivativeInputState) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  penaltyDerivativeInputState.setZero();
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudx[i];
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdx[i].transpose();
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void PenaltyBase<STATE_DIM, INPUT_DIM>::getConstraintViolationSquaredNorm(const scalar_array_t& h, scalar_t& squaredViolation) const {
  squaredViolation = 0;
  for (const scalar_t& hi : h) {
    scalar_t violation = std::max(0.0, -hi);
    squaredViolation += violation * violation;
  }
};

}  // namespace ocs2
