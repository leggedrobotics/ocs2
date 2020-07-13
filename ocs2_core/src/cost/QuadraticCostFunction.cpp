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
void QuadraticCostFunction::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) {
  setCurrentStateAndControl(t, x, u, xNominalIntermediate_, uNominalIntermediate_, xNominalFinal_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void QuadraticCostFunction::setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u,
                                                      const vector_t& xNominalIntermediate, const vector_t& uNominalIntermediate,
                                                      const vector_t& xNominalFinal) {
  // TODO(mspieler): quick hack to fix terminal cost evaluation
  vector_t input = u;
  if (input.size() == 0) {
    input = vector_t::Zero(uNominalIntermediate_.rows());
  }

  CostFunctionBase::setCurrentStateAndControl(t, x, input);

  xNominalIntermediate_ = xNominalIntermediate;
  uNominalIntermediate_ = uNominalIntermediate;
  xNominalFinal_ = xNominalFinal;
  xIntermediateDeviation_ = x - xNominalIntermediate;
  uIntermediateDeviation_ = input - uNominalIntermediate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticCostFunction::getCost() {
  return 0.5 * xIntermediateDeviation_.dot(Q_ * xIntermediateDeviation_) + 0.5 * uIntermediateDeviation_.dot(R_ * uIntermediateDeviation_) +
         uIntermediateDeviation_.dot(P_ * xIntermediateDeviation_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticCostFunction::getCostDerivativeState() {
  return Q_ * xIntermediateDeviation_ + P_.transpose() * uIntermediateDeviation_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticCostFunction::getCostSecondDerivativeState() {
  return Q_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticCostFunction::getCostDerivativeInput() {
  return R_ * uIntermediateDeviation_ + P_ * xIntermediateDeviation_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticCostFunction::getCostSecondDerivativeInput() {
  return R_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticCostFunction::getCostDerivativeInputState() {
  return P_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t QuadraticCostFunction::getTerminalCost() {
  vector_t xFinalDeviation = CostFunctionBase::x_ - xNominalFinal_;
  return 0.5 * xFinalDeviation.dot(QFinal_ * xFinalDeviation);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t QuadraticCostFunction::getTerminalCostDerivativeState() {
  vector_t xFinalDeviation = CostFunctionBase::x_ - xNominalFinal_;
  return QFinal_ * xFinalDeviation;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t QuadraticCostFunction::getTerminalCostSecondDerivativeState() {
  return QFinal_;
}

}  // namespace ocs2
