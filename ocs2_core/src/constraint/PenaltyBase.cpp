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
scalar_t PenaltyBase::penaltyCost(const vector_t& h) const {
  scalar_t penalty = 0;
  for (size_t i = 0; i < h.rows(); i++) {
    penalty += getPenaltyFunctionValue(h(i));
  }
  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation PenaltyBase::penaltyCostQuadraticApproximation(const VectorFunctionQuadraticApproximation& h) const {
  const auto numInequalityConstraints = h.f.rows();
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();

  ScalarFunctionQuadraticApproximation penalty;
  penalty.f = 0.0;
  penalty.dfdx.setZero(stateDim);
  penalty.dfdu.setZero(inputDim);
  penalty.dfdxx.setZero(stateDim, stateDim);
  penalty.dfduu.setZero(inputDim, inputDim);
  penalty.dfdux.setZero(inputDim, stateDim);

  vector_t penaltyDerivative(numInequalityConstraints);
  vector_t penaltySecondDerivative(numInequalityConstraints);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyDerivative(i) = getPenaltyFunctionDerivative(h.f(i));
    penaltySecondDerivative(i) = getPenaltyFunctionSecondDerivative(h.f(i));

    penalty.f += getPenaltyFunctionValue(h.f(i));
    penalty.dfdxx.noalias() += penaltyDerivative(i) * h.dfdxx[i];
    penalty.dfduu.noalias() += penaltyDerivative(i) * h.dfduu[i];
    penalty.dfdux.noalias() += penaltyDerivative(i) * h.dfdux[i];
  }

  penalty.dfdxx.noalias() += h.dfdx.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdx;
  penalty.dfduu.noalias() += h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
  penalty.dfdux.noalias() += h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdx;
  penalty.dfdx = h.dfdx.transpose() * penaltyDerivative;
  penalty.dfdu = h.dfdu.transpose() * penaltyDerivative;

  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t PenaltyBase::constraintViolationSquaredNorm(const vector_t& h) const {
  scalar_t squaredViolation = 0;
  for (size_t i = 0; i < h.rows(); i++) {
    scalar_t violation = std::max(0.0, -h(i));
    squaredViolation += violation * violation;
  }
  return squaredViolation;
}

}  // namespace ocs2
