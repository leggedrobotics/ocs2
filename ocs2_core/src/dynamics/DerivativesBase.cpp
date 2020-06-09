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

#include <ocs2_core/dynamics/DerivativesBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  t_ = t;
  x_ = x;
  u_ = u;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getFlowMapDerivativeTime(vector_t& df) {
  df.setZero(x_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getJumpMapDerivativeTime(vector_t& dg) {
  dg.setZero(x_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getJumpMapDerivativeState(matrix_t& G) {
  G.setIdentity(x_.rows(), x_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getJumpMapDerivativeInput(matrix_t& H) {
  H.setZero(x_.rows(), u_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getGuardSurfacesDerivativeTime(vector_t& D_t_gamma) {
  D_t_gamma = vector_t::Zero(1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getGuardSurfacesDerivativeState(matrix_t& D_x_gamma) {
  D_x_gamma = matrix_t::Zero(1, x_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getGuardSurfacesDerivativeInput(matrix_t& D_u_gamma) {
  D_u_gamma = matrix_t::Zero(1, u_.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void DerivativesBase::getDynamicsCovariance(matrix_t& dynamicsCovariance) {
  dynamicsCovariance.setZero(0, 0);
}

}  // namespace ocs2
