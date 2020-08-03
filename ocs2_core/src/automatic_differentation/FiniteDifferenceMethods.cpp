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

#include <algorithm>

#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t finiteDifferenceDerivative(std::function<vector_t(const vector_t&)> f, const vector_t& x0, scalar_t eps,
                                    bool doubleSidedDerivative) {
  const vector_t f0 = f(x0);
  const size_t varDim = x0.rows();
  const size_t stateDim = f0.rows();
  matrix_t jacobian(stateDim, varDim);

  for (size_t i = 0; i < varDim; i++) {
    // inspired from: http://en.wikipedia.org/wiki/Numerical_differentiation#Practical_considerations_using_floating_point_arithmetic
    scalar_t h = eps * std::max(fabs(x0(i)), 1.0);

    vector_t xPlus = x0;
    xPlus(i) += h;

    if (doubleSidedDerivative) {
      vector_t xMinus = x0;
      xMinus(i) -= h;
      jacobian.col(i) = (f(xPlus) - f(xMinus)) / (2.0 * h);
    } else {
      jacobian.col(i) = (f(xPlus) - f0) / h;
    }
  }

  return jacobian;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t finiteDifferenceDerivativeState(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t eps,
                                         bool doubleSidedDerivative, bool isSecondOrderSystem) {
  auto f = [&](const vector_t& var) -> vector_t { return system.computeFlowMap(t, var, u); };

  matrix_t A = finiteDifferenceDerivative(f, x, eps, doubleSidedDerivative);

  if (isSecondOrderSystem) {
    // Assumes state vector = [x, x_dot]
    A.topLeftCorner(x.rows() / 2, x.rows() / 2).setZero();
    A.topRightCorner(x.rows() / 2, x.rows() / 2).setIdentity();
  }
  return A;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t finiteDifferenceDerivativeInput(ControlledSystemBase& system, scalar_t t, const vector_t& x, const vector_t& u, scalar_t eps,
                                         bool doubleSidedDerivative, bool isSecondOrderSystem) {
  auto f = [&](const vector_t& var) -> vector_t { return system.computeFlowMap(t, x, var); };

  matrix_t B = finiteDifferenceDerivative(f, u, eps, doubleSidedDerivative);

  if (isSecondOrderSystem) {
    // Assumes state vector = [x, x_dot]
    B.topRows(x.rows() / 2).setZero();
  }
  return B;
}

}  // namespace ocs2
