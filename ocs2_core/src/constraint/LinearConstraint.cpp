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
    : ConstraintBase(stateDim, inputDim),
      numStateInputConstraint_(0),
      e_(vector_t()),
      C_(matrix_t()),
      D_(matrix_t()),
      numStateOnlyConstraint_(0),
      h_(vector_t()),
      F_(matrix_t()),
      numStateOnlyFinalConstraint_(0),
      h_f_(vector_t()),
      F_f_(matrix_t()),
      numInequalityConstraint_(0),
      h0_(),
      dhdx_(),
      dhdu_(),
      ddhdxdx_(),
      ddhdudu_(),
      ddhdudx_() {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, size_t numStateInputConstraint, const vector_t& e, const matrix_t& C,
                                   const matrix_t& D, size_t numStateOnlyConstraint, const vector_t& h, const matrix_t& F,
                                   size_t numStateOnlyFinalConstraint, const vector_t& h_f, const matrix_t& F_f)
    : ConstraintBase(stateDim, inputDim),
      numStateInputConstraint_(numStateInputConstraint),
      e_(e),
      C_(C),
      D_(D),
      numStateOnlyConstraint_(numStateOnlyConstraint),
      h_(h),
      F_(F),
      numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint),
      h_f_(h_f),
      F_f_(F_f),
      numInequalityConstraint_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, size_t numStateInputConstraint, const vector_t& e, const matrix_t& C,
                                   const matrix_t& D, size_t numStateOnlyConstraint, const vector_t& h, const matrix_t& F,
                                   size_t numStateOnlyFinalConstraint, const vector_t& h_f, const matrix_t& F_f,
                                   size_t numInequalityConstraint, const scalar_array_t& h0, const vector_array_t& dhdx,
                                   const vector_array_t& dhdu, const matrix_array_t& ddhdxdx, const matrix_array_t& ddhdudu,
                                   const matrix_array_t& ddhdudx)
    : ConstraintBase(stateDim, inputDim),
      numStateInputConstraint_(numStateInputConstraint),
      e_(e),
      C_(C),
      D_(D),
      numStateOnlyConstraint_(numStateOnlyConstraint),
      h_(h),
      F_(F),
      numStateOnlyFinalConstraint_(numStateOnlyFinalConstraint),
      h_f_(h_f),
      F_f_(F_f),
      numInequalityConstraint_(numInequalityConstraint),
      h0_(h0),
      dhdx_(dhdx),
      dhdu_(dhdu),
      ddhdxdx_(ddhdxdx),
      ddhdudu_(ddhdudu),
      ddhdudx_(ddhdudx) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint* LinearConstraint::clone() const {
  return new LinearConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getConstraint1(vector_t& g1) {
  g1 = e_ + C_ * ConstraintBase::x_ + D_ * ConstraintBase::u_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearConstraint::numStateInputConstraint(const scalar_t& time) {
  return numStateInputConstraint_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getConstraint2(vector_t& g2) {
  g2 = h_ + F_ * ConstraintBase::x_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearConstraint::numStateOnlyConstraint(const scalar_t& time) {
  return numStateOnlyConstraint_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraint(scalar_array_t& h) {
  h.clear();
  for (size_t i = 0; i < numInequalityConstraint_; i++) {
    h.emplace_back(h0_[i] + dhdx_[i].dot(ConstraintBase::x_) + dhdu_[i].dot(ConstraintBase::u_) +
                   0.5 * ConstraintBase::x_.dot(ddhdxdx_[i] * ConstraintBase::x_) +
                   0.5 * ConstraintBase::u_.dot(ddhdudu_[i] * ConstraintBase::u_) +
                   ConstraintBase::u_.dot(ddhdudx_[i] * ConstraintBase::x_));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearConstraint::numInequalityConstraint(const scalar_t& time) {
  return numInequalityConstraint_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getFinalConstraint2(vector_t& g2Final) {
  g2Final = h_f_ + F_f_ * ConstraintBase::x_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t LinearConstraint::numStateOnlyFinalConstraint(const scalar_t& time) {
  return numStateOnlyFinalConstraint_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getConstraint1DerivativesState(matrix_t& C) {
  C = C_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getConstraint1DerivativesControl(matrix_t& D) {
  D = D_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getConstraint2DerivativesState(matrix_t& F) {
  F = F_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraintDerivativesState(vector_array_t& dhdx) {
  dhdx.clear();
  for (size_t i = 0; i < numInequalityConstraint_; i++) {
    dhdx.push_back(dhdx_[i] + ddhdxdx_[i] * ConstraintBase::x_ + ddhdudx_[i].transpose() * ConstraintBase::u_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraintDerivativesInput(vector_array_t& dhdu) {
  dhdu.clear();
  for (size_t i = 0; i < numInequalityConstraint_; i++) {
    dhdu.push_back(dhdu_[i] + ddhdudu_[i] * ConstraintBase::u_ + ddhdudx_[i] * ConstraintBase::x_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraintSecondDerivativesState(matrix_array_t& ddhdxdx) {
  ddhdxdx = ddhdxdx_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraintSecondDerivativesInput(matrix_array_t& ddhdudu) {
  ddhdudu = ddhdudu_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getInequalityConstraintDerivativesInputState(matrix_array_t& ddhdudx) {
  ddhdudx = ddhdudx_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearConstraint::getFinalConstraint2DerivativesState(matrix_t& F_f) {
  F_f = F_f_;
}

}  // namespace ocs2
