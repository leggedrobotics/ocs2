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
LinearSystemDynamics::LinearSystemDynamics(const matrix_t& A, const matrix_t& B, const matrix_t& G /*= matrix_t()*/,
                                           const matrix_t& H /*= matrix_t()*/)
    : SystemDynamicsBase(), A_(A), B_(B), G_(G), H_(H) {
  if (G_.size() == 0) {
    G_ = matrix_t::Zero(A_.rows(), A_.cols());
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
void LinearSystemDynamics::computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) {
  dxdt = A_ * x + B_ * u;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::computeJumpMap(const scalar_t& t, const vector_t& x, vector_t& xp) {
  xp = G_ * x;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  SystemDynamicsBase::setCurrentStateAndControl(t, x, u);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::getFlowMapDerivativeState(matrix_t& A) {
  A = A_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::getFlowMapDerivativeInput(matrix_t& B) {
  B = B_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::getJumpMapDerivativeState(matrix_t& G) {
  G = G_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearSystemDynamics::getJumpMapDerivativeInput(matrix_t& H) {
  H = H_;
}

}  // namespace ocs2
