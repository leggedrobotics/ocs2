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

#include <ocs2_core/cost/QuadraticStateInputCost.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateInputCost::QuadraticStateInputCost(matrix_t Q, matrix_t R, matrix_t P /* = matrix_t() */)
    : Q_(std::move(Q)), R_(std::move(R)), P_(std::move(P)) {
  if (P_.size() == 0) {
    P_ = matrix_t::Zero(R_.rows(), Q_.rows());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticStateInputCost* QuadraticStateInputCost::clone() const {
  return new QuadraticStateInputCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticStateInputCost::getValue(scalar_t t, const vector_t& x, const vector_t& u,
                                           const CostDesiredTrajectories& desiredTrajectory) const {
  const vector_t xDeviation = x - desiredTrajectory.getDesiredState(t);
  const vector_t uDeviation = u - desiredTrajectory.getDesiredInput(t);
  return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation) + uDeviation.dot(P_ * xDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation QuadraticStateInputCost::getQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, const CostDesiredTrajectories& desiredTrajectory) const {
  const vector_t xDeviation = x - desiredTrajectory.getDesiredState(t);
  const vector_t uDeviation = u - desiredTrajectory.getDesiredInput(t);
  const vector_t qDeviation = Q_ * xDeviation;
  const vector_t rDeviation = R_ * uDeviation;
  const vector_t pDeviation = P_ * xDeviation;

  ScalarFunctionQuadraticApproximation L;
  L.f = 0.5 * xDeviation.dot(qDeviation) + 0.5 * uDeviation.dot(rDeviation) + uDeviation.dot(pDeviation);
  L.dfdx = qDeviation;
  L.dfdx.noalias() += P_.transpose() * uDeviation;
  L.dfdu = rDeviation + pDeviation;
  L.dfdxx = Q_;
  L.dfdux = P_;
  L.dfduu = R_;
  return L;
}

}  // namespace ocs2
