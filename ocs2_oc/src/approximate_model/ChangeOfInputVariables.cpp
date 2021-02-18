/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include "ocs2_oc/approximate_model/ChangeOfInputVariables.h"

namespace ocs2 {

void changeOfInputVariables(ScalarFunctionQuadraticApproximation& quadraticApproximation, const matrix_t& Pu, const matrix_t& Px,
                            const vector_t& u0) {
  /*
   * 3 temporaries are needed in any branch because Pu is non-zero and:
   *  - new P contains a product Pu'*P
   *  - new R contains a product R*Pu
   *  - new r contains a product Pu'*r
   *  Other than that, the adaptation can be done in-place
   *
   *  The terms of the quadratic functions have the following notation:
   *  dfdxx = Q, dfdux = P, dfduu = R, dfdx = q, dfdu = r, f = c.
   */
  const bool hasPx(Px.size() > 0);
  const bool hasu0(u0.size() > 0);

  // Shared term number 1
  matrix_t P_plus_R_Px = quadraticApproximation.dfdux;
  if (hasPx) {
    P_plus_R_Px.noalias() += quadraticApproximation.dfduu * Px;
  }  // else added term is zero

  // Shared term number 2
  vector_t r_plus_R_u0 = quadraticApproximation.dfdu;
  if (hasu0) {
    r_plus_R_u0.noalias() += quadraticApproximation.dfduu * u0;
  }  // else added term is zero

  // Q = Q + P'*Px + Px'*P + Px'*R*Px = Q + P'*Px + Px'*(P + R*Px)
  if (hasPx) {
    quadraticApproximation.dfdxx.noalias() += quadraticApproximation.dfdux.transpose() * Px;  // Before adapting dfdux!
    quadraticApproximation.dfdxx.noalias() += Px.transpose() * P_plus_R_Px;
  }  // else Q remains unaltered

  // q = q + P' * u0 + Px' (R*u0 + r)
  if (hasu0) {
    quadraticApproximation.dfdx.noalias() += quadraticApproximation.dfdux.transpose() * u0;  // Before adapting dfdux!
  }
  if (hasPx) {
    quadraticApproximation.dfdx.noalias() += Px.transpose() * r_plus_R_u0;
  }

  // c = c + r'*u0 + 1/2*u0'*R*u0 = 1/2*u0'((R*u0 + r) + r)
  if (hasu0) {
    quadraticApproximation.f += 0.5 * u0.dot(r_plus_R_u0 + quadraticApproximation.dfdu);  // Before adapting dfdu!
  }

  // P = Pu'*P + Pu'*R*Px = Pu'*(P + R*Px)
  quadraticApproximation.dfdux.noalias() = Pu.transpose() * P_plus_R_Px;

  // R = Pu' * R * Pu
  matrix_t R_Pu = quadraticApproximation.dfduu * Pu;  // make the required temporary explicit, to save it in the second multiplication
  quadraticApproximation.dfduu.noalias() = Pu.transpose() * R_Pu;

  // r = Pu' * (R*u0 + r)
  quadraticApproximation.dfdu.noalias() = Pu.transpose() * r_plus_R_u0;
}

void changeOfInputVariables(VectorFunctionLinearApproximation& linearApproximation, const matrix_t& Pu, const matrix_t& Px,
                            const vector_t& u0) {
  const bool hasPx(Px.size() > 0);
  const bool hasu0(u0.size() > 0);

  // A = A + B*Px
  if (hasPx) {
    linearApproximation.dfdx.noalias() += linearApproximation.dfdu * Px;  // Before adapting dfdu!
  }

  // b = b + B*u0
  if (hasu0) {
    linearApproximation.f.noalias() += linearApproximation.dfdu * u0;  // Before adapting dfdu!
  }

  // B = B*Pu
  linearApproximation.dfdu = linearApproximation.dfdu * Pu;  // temporary matrix unavoidable
}

}  // namespace ocs2
