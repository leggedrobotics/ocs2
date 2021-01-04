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

#include <cassert>

#include <ocs2_core/soft_constraint/SoftConstraintPenalty.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SoftConstraintPenalty::SoftConstraintPenalty(std::vector<std::unique_ptr<PenaltyFunctionBase>> penaltyFunctionPtrArray)
    : penaltyFunctionPtrArray_(std::move(penaltyFunctionPtrArray)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SoftConstraintPenalty::SoftConstraintPenalty(size_t numConstraints, std::unique_ptr<PenaltyFunctionBase> penaltyFunctionPtr) {
  for (size_t i = 0; i < numConstraints; i++) {
    penaltyFunctionPtrArray_.emplace_back(penaltyFunctionPtr->clone());
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SoftConstraintPenalty::SoftConstraintPenalty(const SoftConstraintPenalty& other) {
  for (size_t i = 0; i < other.penaltyFunctionPtrArray_.size(); i++) {
    penaltyFunctionPtrArray_.emplace_back(other.penaltyFunctionPtrArray_[i]->clone());
  }  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SoftConstraintPenalty::getValue(const vector_t& h) const {
  const auto numInequalityConstraints = h.rows();
  assert(penaltyFunctionPtrArray_.size() == numInequalityConstraints);
  scalar_t penalty = 0;
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penalty += penaltyFunctionPtrArray_[i]->getValue(h(i));
  }
  return penalty;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SoftConstraintPenalty::getQuadraticApproximation(const VectorFunctionLinearApproximation& h) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = getPenaltyValue1stDev2ndDev(h.f);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  auto penaltyApproximation = ScalarFunctionQuadraticApproximation::Zero(stateDim, inputDim);

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() = h.dfdx.transpose() * penaltySecondDev_dhdx;
  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() = h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() = h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation SoftConstraintPenalty::getQuadraticApproximation(const VectorFunctionQuadraticApproximation& h) const {
  const auto stateDim = h.dfdx.cols();
  const auto inputDim = h.dfdu.cols();

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative, penaltySecondDerivative;
  std::tie(penaltyValue, penaltyDerivative, penaltySecondDerivative) = getPenaltyValue1stDev2ndDev(h.f);
  const matrix_t penaltySecondDev_dhdx = penaltySecondDerivative.asDiagonal() * h.dfdx;

  auto penaltyApproximation = ScalarFunctionQuadraticApproximation::Zero(stateDim, inputDim);
  for (size_t i = 0; i < penaltyDerivative.rows(); i++) {
    penaltyApproximation.dfdxx.noalias() += penaltyDerivative(i) * h.dfdxx[i];
    penaltyApproximation.dfduu.noalias() += penaltyDerivative(i) * h.dfduu[i];
    penaltyApproximation.dfdux.noalias() += penaltyDerivative(i) * h.dfdux[i];
  }

  penaltyApproximation.f = penaltyValue;
  penaltyApproximation.dfdx.noalias() = h.dfdx.transpose() * penaltyDerivative;
  penaltyApproximation.dfdxx.noalias() += h.dfdx.transpose() * penaltySecondDev_dhdx;
  if (inputDim > 0) {
    penaltyApproximation.dfdu.noalias() = h.dfdu.transpose() * penaltyDerivative;
    penaltyApproximation.dfdux.noalias() += h.dfdu.transpose() * penaltySecondDev_dhdx;
    penaltyApproximation.dfduu.noalias() += h.dfdu.transpose() * penaltySecondDerivative.asDiagonal() * h.dfdu;
  }

  return penaltyApproximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::tuple<scalar_t, vector_t, vector_t> SoftConstraintPenalty::getPenaltyValue1stDev2ndDev(const vector_t& h) const {
  const auto numInequalityConstraints = h.rows();
  assert(penaltyFunctionPtrArray_.size() == numInequalityConstraints);

  scalar_t penaltyValue = 0.0;
  vector_t penaltyDerivative(numInequalityConstraints);
  vector_t penaltySecondDerivative(numInequalityConstraints);
  for (size_t i = 0; i < numInequalityConstraints; i++) {
    penaltyValue += penaltyFunctionPtrArray_[i]->getValue(h(i));
    penaltyDerivative(i) = penaltyFunctionPtrArray_[i]->getDerivative(h(i));
    penaltySecondDerivative(i) = penaltyFunctionPtrArray_[i]->getSecondDerivative(h(i));
  }  // end of i loop

  return {penaltyValue, penaltyDerivative, penaltySecondDerivative};
}

}  // namespace ocs2
