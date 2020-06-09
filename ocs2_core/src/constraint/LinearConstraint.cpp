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

#include <ocs2_core/constraint/LinearConstraint.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim)
    : ConstraintBase(stateDim, inputDim), e_(0), C_(0, stateDim), D_(0, inputDim), h_(0), F_(0, stateDim), h_f_(0), F_f_(0, stateDim) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, vector_t e, matrix_t C, matrix_t D, vector_t h, matrix_t F,
                                   vector_t h_f, matrix_t F_f)
    : ConstraintBase(stateDim, inputDim),
      e_(std::move(e)),
      C_(std::move(C)),
      D_(std::move(D)),
      h_(std::move(h)),
      F_(std::move(F)),
      h_f_(std::move(h_f)),
      F_f_(std::move(F_f)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, vector_t e, matrix_t C, matrix_t D, vector_t h, matrix_t F,
                                   vector_t h_f, matrix_t F_f, scalar_array_t h0, vector_array_t dhdx, vector_array_t dhdu,
                                   matrix_array_t ddhdxdx, matrix_array_t ddhdudu, matrix_array_t ddhdudx)
    : ConstraintBase(stateDim, inputDim),
      e_(std::move(e)),
      C_(std::move(C)),
      D_(std::move(D)),
      h_(std::move(h)),
      F_(std::move(F)),
      h_f_(std::move(h_f)),
      F_f_(std::move(F_f)),
      h0_(std::move(h0)),
      dhdx_(std::move(dhdx)),
      dhdu_(std::move(dhdu)),
      ddhdxdx_(std::move(ddhdxdx)),
      ddhdudu_(std::move(ddhdudu)),
      ddhdudx_(std::move(ddhdudx)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint* LinearConstraint::clone() const {
  return new LinearConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::getStateInputEqualityConstraint() {
  return e_ + C_ * ConstraintBase::x_ + D_ * ConstraintBase::u_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::getStateEqualityConstraint() {
  return h_ + F_ * ConstraintBase::x_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_array_t LinearConstraint::getInequalityConstraint() {
  scalar_array_t h(h0_.size());
  for (size_t i = 0; i < h0_.size(); i++) {
    h[i] = h0_[i] + dhdx_[i].dot(ConstraintBase::x_) + dhdu_[i].dot(ConstraintBase::u_) +
           0.5 * ConstraintBase::x_.dot(ddhdxdx_[i] * ConstraintBase::x_) + 0.5 * ConstraintBase::u_.dot(ddhdudu_[i] * ConstraintBase::u_) +
           ConstraintBase::u_.dot(ddhdudx_[i] * ConstraintBase::x_);
  }
  return h;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::getFinalStateEqualityConstraint() {
  return h_f_ + F_f_ * ConstraintBase::x_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LinearConstraint::getStateInputEqualityConstraintDerivativesState() {
  return C_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LinearConstraint::getStateInputEqualityConstraintDerivativesInput() {
  return D_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LinearConstraint::getStateEqualityConstraintDerivativesState() {
  return F_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t LinearConstraint::getInequalityConstraintDerivativesState() {
  vector_array_t dhdx;
  dhdx.reserve(h0_.size());
  for (size_t i = 0; i < h0_.size(); i++) {
    dhdx.emplace_back(dhdx_[i] + ddhdxdx_[i] * ConstraintBase::x_ + ddhdudx_[i].transpose() * ConstraintBase::u_);
  }
  return dhdx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_array_t LinearConstraint::getInequalityConstraintDerivativesInput() {
  vector_array_t dhdu;
  dhdu.reserve(h0_.size());
  for (size_t i = 0; i < h0_.size(); i++) {
    dhdu.emplace_back(dhdu_[i] + ddhdudu_[i] * ConstraintBase::u_ + ddhdudx_[i] * ConstraintBase::x_);
  }
  return dhdu;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t LinearConstraint::getInequalityConstraintSecondDerivativesState() {
  return ddhdxdx_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t LinearConstraint::getInequalityConstraintSecondDerivativesInput() {
  return ddhdudu_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_array_t LinearConstraint::getInequalityConstraintDerivativesInputState() {
  return ddhdudx_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LinearConstraint::getFinalStateEqualityConstraintDerivativesState() {
  return F_f_;
}

}  // namespace ocs2
