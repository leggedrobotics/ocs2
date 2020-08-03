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

#include <ocs2_core/dynamics/LinearSystemDynamics.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearSystemDynamics::LinearSystemDynamics(matrix_t A, matrix_t B, matrix_t G /*= matrix_t()*/, matrix_t H /*= matrix_t()*/)
    : A_(std::move(A)), B_(std::move(B)), G_(std::move(G)), H_(std::move(H)) {
  if (G_.size() == 0) {
    G_ = matrix_t::Zero(A_.rows(), A_.rows());
  }
  if (H_.size() == 0) {
    H_ = matrix_t::Zero(A_.rows(), B_.cols());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearSystemDynamics* LinearSystemDynamics::clone() const {
  return new LinearSystemDynamics(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearSystemDynamics::computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) {
  return A_ * x + B_ * u;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearSystemDynamics::computeJumpMap(scalar_t t, const vector_t& x) {
  return G_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = A_;
  approximation.dfdu = B_;
  approximation.f = A_ * x + B_ * u;
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::jumpMapLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
  VectorFunctionLinearApproximation approximation;
  approximation.dfdx = G_;
  approximation.dfdu = H_;
  approximation.f = G_ * x;
  return approximation;
}

}  // namespace ocs2
