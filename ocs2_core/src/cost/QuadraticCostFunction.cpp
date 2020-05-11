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

#include <ocs2_core/cost/QuadraticCostFunction.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticCostFunction::QuadraticCostFunction(const matrix_t& Q, const matrix_t& R, const vector_t& xNominalIntermediate,
                                             const vector_t& uNominalIntermediate, const matrix_t& QFinal, const vector_t& xNominalFinal,
                                             const matrix_t& P /* = matrix_t() */)
    : Q_(Q),
      R_(R),
      P_(P),
      QFinal_(QFinal),
      xNominalIntermediate_(xNominalIntermediate),
      uNominalIntermediate_(uNominalIntermediate),
      xNominalFinal_(xNominalFinal) {
  if (P_.size() == 0) {
    P_ = matrix_t::Zero(R.rows(), Q.rows());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
QuadraticCostFunction* QuadraticCostFunction::clone() const {
  return new QuadraticCostFunction(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) {
  setCurrentStateAndControl(t, x, u, xNominalIntermediate_, uNominalIntermediate_, xNominalFinal_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u,
                                                      const vector_t& xNominalIntermediate, const vector_t& uNominalIntermediate,
                                                      const vector_t& xNominalFinal) {
  CostFunctionBase::setCurrentStateAndControl(t, x, u);

  xNominalIntermediate_ = xNominalIntermediate;
  uNominalIntermediate_ = uNominalIntermediate;
  xNominalFinal_ = xNominalFinal;
  xIntermediateDeviation_ = x - xNominalIntermediate;
  uIntermediateDeviation_ = u - uNominalIntermediate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCost(scalar_t& L) {
  L = 0.5 * xIntermediateDeviation_.dot(Q_ * xIntermediateDeviation_) + 0.5 * uIntermediateDeviation_.dot(R_ * uIntermediateDeviation_) +
      uIntermediateDeviation_.dot(P_ * xIntermediateDeviation_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCostDerivativeState(vector_t& dLdx) {
  dLdx = Q_ * xIntermediateDeviation_ + P_.transpose() * uIntermediateDeviation_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) {
  dLdxx = Q_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCostDerivativeInput(vector_t& dLdu) {
  dLdu = R_ * uIntermediateDeviation_ + P_ * xIntermediateDeviation_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) {
  dLduu = R_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getIntermediateCostDerivativeInputState(matrix_t& dLdux) {
  dLdux = P_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getTerminalCost(scalar_t& cost) {
  vector_t xFinalDeviation = CostFunctionBase::x_ - xNominalFinal_;
  cost = 0.5 * xFinalDeviation.dot(QFinal_ * xFinalDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getTerminalCostDerivativeState(vector_t& dPhidx) {
  vector_t xFinalDeviation = CostFunctionBase::x_ - xNominalFinal_;
  dPhidx = QFinal_ * xFinalDeviation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) {
  dPhidxx = QFinal_;
}

}  // namespace ocs2
