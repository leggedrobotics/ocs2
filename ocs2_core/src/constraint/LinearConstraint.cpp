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
    : e_(0),
      C_(0, stateDim),
      D_(0, inputDim),
      h_(0),
      F_(0, stateDim),
      h_f_(0),
      F_f_(0, stateDim),
      h0_(0),
      dhdx_(0, stateDim),
      dhdu_(0, inputDim),
      dhdxx_(0),
      dhduu_(0),
      dhdux_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(vector_t e, matrix_t C, matrix_t D, vector_t h, matrix_t F, vector_t h_f, matrix_t F_f)
    : e_(std::move(e)),
      C_(std::move(C)),
      D_(std::move(D)),
      h_(std::move(h)),
      F_(std::move(F)),
      h_f_(std::move(h_f)),
      F_f_(std::move(F_f)),
      h0_(0),
      dhdx_(0, C.cols()),
      dhdu_(0, D.cols()),
      dhdxx_(0),
      dhduu_(0),
      dhdux_(0) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint::LinearConstraint(vector_t e, matrix_t C, matrix_t D, vector_t h, matrix_t F, vector_t h_f, matrix_t F_f, vector_t h0,
                                   matrix_t dhdx, matrix_t dhdu, matrix_array_t dhdxx, matrix_array_t dhduu, matrix_array_t dhdux)
    : e_(std::move(e)),
      C_(std::move(C)),
      D_(std::move(D)),
      h_(std::move(h)),
      F_(std::move(F)),
      h_f_(std::move(h_f)),
      F_f_(std::move(F_f)),
      h0_(std::move(h0)),
      dhdx_(std::move(dhdx)),
      dhdu_(std::move(dhdu)),
      dhdxx_(std::move(dhdxx)),
      dhduu_(std::move(dhduu)),
      dhdux_(std::move(dhdux)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearConstraint* LinearConstraint::clone() const {
  return new LinearConstraint(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  return e_ + C_ * x + D_ * u;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::stateEqualityConstraint(scalar_t t, const vector_t& x) {
  return h_ + F_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u) {
  vector_t h = h0_ + dhdx_ * x + dhdu_ * u;
  for (size_t i = 0; i < h.rows(); i++) {
    h[i] += 0.5 * x.dot(dhdxx_[i] * x) + 0.5 * u.dot(dhduu_[i] * u) + u.dot(dhdux_[i] * x);
  }
  return h;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearConstraint::finalStateEqualityConstraint(scalar_t t, const vector_t& x) {
  return h_f_ + F_f_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearConstraint::stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                                    const vector_t& u) {
  VectorFunctionLinearApproximation g;
  g.f = e_ + C_ * x + D_ * u;
  g.dfdx = C_;
  g.dfdu = D_;
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearConstraint::stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  VectorFunctionLinearApproximation g;
  g.f = h_ + F_ * x;
  g.dfdx = F_;
  return g;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation LinearConstraint::inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x,
                                                                                                  const vector_t& u) {
  VectorFunctionQuadraticApproximation h;
  h.f = h0_ + dhdx_ * x + dhdu_ * u;
  h.dfdx = dhdx_;
  h.dfdu = dhdu_;
  for (size_t i = 0; i < h.f.rows(); i++) {
    h.f[i] += 0.5 * x.dot(dhdxx_[i] * x) + 0.5 * u.dot(dhduu_[i] * u) + u.dot(dhdux_[i] * x);
    h.dfdx.row(i) += (dhdxx_[i] * x + dhdux_[i].transpose() * u).transpose();
    h.dfdu.row(i) += (dhduu_[i] * u + dhdux_[i] * x).transpose();
  }
  h.dfdxx = dhdxx_;
  h.dfdux = dhdux_;
  h.dfduu = dhduu_;
  return h;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearConstraint::finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x) {
  VectorFunctionLinearApproximation gf;
  gf.f = h_f_ + F_f_ * x;
  gf.dfdx = F_f_;
  return gf;
}

}  // namespace ocs2
