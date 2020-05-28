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
      e_(vector_t(0)),
      C_(matrix_t(0, stateDim)),
      D_(matrix_t(0, inputDim)),
      h_(vector_t(0)),
      F_(matrix_t(0, stateDim)),
      h_f_(vector_t(0)),
      F_f_(matrix_t(0, stateDim)),
      h0_(0),
      dhdx_(0),
      dhdu_(0),
      ddhdxdx_(0),
      ddhdudu_(0),
      ddhdudx_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, const vector_t& e, const matrix_t& C, const matrix_t& D,
                                   const vector_t& h, const matrix_t& F, const vector_t& h_f, const matrix_t& F_f)
    : ConstraintBase(stateDim, inputDim),
      e_(e),
      C_(C),
      D_(D),
      h_(h),
      F_(F),
      h_f_(h_f),
      F_f_(F_f),
      h0_(0),
      dhdx_(0),
      dhdu_(0),
      ddhdxdx_(0),
      ddhdudu_(0),
      ddhdudx_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(size_t stateDim, size_t inputDim, const vector_t& e, const matrix_t& C, const matrix_t& D,
                                   const vector_t& h, const matrix_t& F, const vector_t& h_f, const matrix_t& F_f, const scalar_array_t& h0,
                                   const vector_array_t& dhdx, const vector_array_t& dhdu, const matrix_array_t& ddhdxdx,
                                   const matrix_array_t& ddhdudu, const matrix_array_t& ddhdudx)
    : ConstraintBase(stateDim, inputDim),
      e_(e),
      C_(C),
      D_(D),
      h_(h),
      F_(F),
      h_f_(h_f),
      F_f_(F_f),
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
