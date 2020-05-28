/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include <ocs2_core/constraint/ConstraintBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBase::ConstraintBase(size_t stateDim, size_t inputDim) : stateDim_(stateDim), inputDim_(inputDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ConstraintBase::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  t_ = t;
  x_ = x;
  u_ = u;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ConstraintBase* ConstraintBase::clone() const {
  return new ConstraintBase(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::getStateInputEqualityConstraint() {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::getStateEqualityConstraint() {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_array_t ConstraintBase::getInequalityConstraint() {
  return scalar_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ConstraintBase::getFinalStateEqualityConstraint() {
  return vector_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBase::getStateInputEqualityConstraintDerivativesState() {
  return matrix_t(0, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBase::getStateInputEqualityConstraintDerivativesInput() {
  return matrix_t(0, inputDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t ConstraintBase::getStateInputEqualityConstraintDerivativesEventTimes() {
  return vector_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBase::getStateEqualityConstraintDerivativesState() {
  return matrix_t(0, stateDim_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t ConstraintBase::getInequalityConstraintDerivativesState() {
  return vector_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t ConstraintBase::getInequalityConstraintDerivativesInput() {
  return vector_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t ConstraintBase::getInequalityConstraintSecondDerivativesState() {
  return matrix_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t ConstraintBase::getInequalityConstraintSecondDerivativesInput() {
  return matrix_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t ConstraintBase::getInequalityConstraintDerivativesInputState() {
  return matrix_array_t(0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t ConstraintBase::getFinalStateEqualityConstraintDerivativesState() {
  return matrix_t(0, stateDim_);
}

}  // namespace ocs2
