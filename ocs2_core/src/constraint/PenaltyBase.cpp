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
scalar_t PenaltyBase::getPenaltyCost(const scalar_array_t& h) const {
  scalar_t penalty = 0;
  for (const scalar_t& hi : h) {
    penalty += getPenaltyFunctionValue(hi);
  }
  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PenaltyBase::getPenaltyCostDerivativeState(const scalar_array_t& h, const vector_array_t& dhdx) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  const auto stateDim = dhdx.front().size();
  vector_t penaltyDerivativeState = vector_t::Zero(stateDim);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdx[i];
  }
  return penaltyDerivativeState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t PenaltyBase::getPenaltyCostDerivativeInput(const scalar_array_t& h, const vector_array_t& dhdu) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  const auto inputDim = dhdu.front().size();
  vector_t penaltyDerivativeInput = vector_t::Zero(inputDim);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * dhdu[i];
  }
  return penaltyDerivativeInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PenaltyBase::getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const vector_array_t& dhdx,
                                                          const matrix_array_t& ddhdxdx) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != ddhdxdx.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  const auto stateDim = dhdx.front().size();
  matrix_t penaltySecondDerivativeState = matrix_t::Zero(stateDim, stateDim);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdxdx[i];
    penaltySecondDerivativeState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdx[i] * dhdx[i].transpose();
  }
  return penaltySecondDerivativeState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PenaltyBase::getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const vector_array_t& dhdu,
                                                          const matrix_array_t& ddhdudu) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdu.size() || numInequalityConstraints != ddhdudu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  const auto inputDim = dhdu.front().size();
  matrix_t penaltySecondDerivativeInput = matrix_t::Zero(inputDim, inputDim);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudu[i];
    penaltySecondDerivativeInput.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdu[i].transpose();
  }
  return penaltySecondDerivativeInput;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t PenaltyBase::getPenaltyCostDerivativeInputState(const scalar_array_t& h, const vector_array_t& dhdx, const vector_array_t& dhdu,
                                                         const matrix_array_t& ddhdudx) const {
  const size_t numInequalityConstraints = h.size();
  if (numInequalityConstraints != dhdx.size() || numInequalityConstraints != dhdu.size()) {
    throw std::runtime_error("The inequality constraint should have the same size as it derivative.");
  }

  const auto stateDim = dhdx.front().size();
  const auto inputDim = dhdu.front().size();
  matrix_t penaltyDerivativeInputState = matrix_t::Zero(inputDim, stateDim);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionDerivative(h[i]) * ddhdudx[i];
    penaltyDerivativeInputState.noalias() += getPenaltyFunctionSecondDerivative(h[i]) * dhdu[i] * dhdx[i].transpose();
  }
  return penaltyDerivativeInputState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PenaltyBase::getConstraintViolationSquaredNorm(const scalar_array_t& h) const {
  scalar_t squaredViolation = 0;
  for (const scalar_t& hi : h) {
    scalar_t violation = std::max(0.0, -hi);
    squaredViolation += violation * violation;
  }
  return squaredViolation;
}

}  // namespace ocs2
