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
LinearSystemDynamics::LinearSystemDynamics(matrix_t A, matrix_t B, matrix_t G /*= matrix_t()*/)
    : A_(std::move(A)), B_(std::move(B)), G_(std::move(G)) {
  if (G_.size() == 0) {
    G_.setIdentity(A_.rows(), A_.rows());
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
vector_t LinearSystemDynamics::computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) {
  vector_t f = A_ * x;
  f.noalias() += B_ * u;
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearSystemDynamics::computeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) {
  return G_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                            const PreComputation&) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = A_ * x;
  approximation.f.noalias() += B_ * u;
  approximation.dfdx = A_;
  approximation.dfdu = B_;
  return approximation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation LinearSystemDynamics::jumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation&) {
  VectorFunctionLinearApproximation approximation;
  approximation.f = G_ * x;
  approximation.dfdx = G_;
  approximation.dfdu.setZero(A_.rows(), 0);
  return approximation;
}

}  // namespace ocs2
