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

#include <ocs2_core/dynamics/TransferFunctionBase.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline void padeApproximation(scalar_t timeDelay, vector_t& numCoefficients, vector_t& denCoefficients, size_t numZeros, size_t numPoles) {
  numCoefficients.resize(numZeros + 1);
  denCoefficients.resize(numPoles + 1);

  if (numZeros == 0 && numPoles == 1) {
    numCoefficients << 1.0;
    denCoefficients << timeDelay, 1.0;
    return;
  }

  if (numZeros == 1 && numPoles == 0) {
    numCoefficients << -timeDelay, 1.0;
    denCoefficients << 1.0;
    return;
  }

  if (numZeros == 1 && numPoles == 1) {
    numCoefficients << -0.5 * timeDelay, 1.0;
    denCoefficients << 0.5 * timeDelay, 1.0;
    return;
  }

  std::runtime_error("padeApproximation not implemented for nZeros=" + std::to_string(numZeros) + " nPoles= " + std::to_string(numPoles));
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline vector_t multiplyPolynomials(const vector_t& p_lhs, const vector_t& p_rhs) {
  vector_t p_result(p_lhs.size() + p_rhs.size() - 1);
  p_result.setZero();
  for (int i = 0; i < p_lhs.size(); i++) {
    for (int j = 0; j < p_lhs.size(); j++) {
      p_result(i + j) += p_lhs(i) * p_rhs(j);
    }
  }
  return p_result;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TransferFunctionBase::TransferFunctionBase(vector_t numCoefficients, vector_t denCoefficients, scalar_t timedelay, bool balance)
    : numCoefficients_(std::move(numCoefficients)),
      denCoefficients_(std::move(denCoefficients)),
      timeDelay_(timedelay),
      balance_(balance){};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TransferFunctionBase::absorbDelay(size_t numZeros, size_t numPoles) {
  if (timeDelay_ > delayTol_) {
    // Approximate delay
    vector_t padeNum, padeDen;
    ocs2::padeApproximation(timeDelay_, padeNum, padeDen, numZeros, numPoles);

    // multiply approximated delay
    numCoefficients_ = multiplyPolynomials(numCoefficients_, padeNum);
    denCoefficients_ = multiplyPolynomials(denCoefficients_, padeDen);
  }
  delayAbsorbed_ = true;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TransferFunctionBase::normalize() {
  scalar_t scaling = denCoefficients_(0);
  numCoefficients_ /= scaling;
  denCoefficients_ /= scaling;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TransferFunctionBase::getStateSpace(matrix_t& A, matrix_t& B, matrix_t& C, matrix_t& D) {
  if (numCoefficients_.size() > denCoefficients_.size()) {
    throw std::runtime_error("Transfer function must be proper to convert to a state space model");
  }

  // Absorb delay and normalize
  if (!delayAbsorbed_) {  // Default approximation of time delay
    this->absorbDelay(1, 1);
  }
  this->normalize();

  size_t numStates = denCoefficients_.size() - 1;
  size_t numInputs = 1;
  size_t numOutputs = 1;

  A.resize(numStates, numStates);
  B.resize(numStates, numInputs);
  C.resize(numOutputs, numStates);
  D.resize(numOutputs, numInputs);

  // prepend zeros to numerator
  vector_t numExtended(denCoefficients_.size());
  numExtended.setZero();
  numExtended.tail(numCoefficients_.size()) = numCoefficients_;

  // Create strictly proper transfer function
  D(0) = numExtended(0);
  numExtended -= denCoefficients_ * D(0);

  if (numStates > 0) {
    A << -denCoefficients_.tail(numStates).transpose(), matrix_t::Identity(numStates - 1, numStates);
    B << 1.0, vector_t::Zero(numStates - 1);
    C << numExtended.tail(numStates).transpose();

    if (balance_) {
      matrix_t T(numStates, numStates);
      T.setZero();
      T.diagonal() = denCoefficients_.tail(numStates).cwiseSqrt();
      T(0, 0) = 1.0;
      A = T * A.eval() * T.inverse();
      B = T * B.eval();
      C = C.eval() * T.inverse();
    }
  }
}

}  // namespace ocs2
