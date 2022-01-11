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

#include <ocs2_core/Types.h>

#include <ocs2_core/misc/LinearAlgebra.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation::ScalarFunctionQuadraticApproximation(size_t nx, size_t nu) {
  resize(nx, nu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation& ScalarFunctionQuadraticApproximation::operator+=(const ScalarFunctionQuadraticApproximation& rhs) {
  f += rhs.f;
  dfdx += rhs.dfdx;
  dfdu += rhs.dfdu;
  dfdxx += rhs.dfdxx;
  dfdux += rhs.dfdux;
  dfduu += rhs.dfduu;
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation& ScalarFunctionQuadraticApproximation::operator*=(scalar_t scalar) {
  f *= scalar;
  dfdx *= scalar;
  dfdu *= scalar;
  dfdxx *= scalar;
  dfdux *= scalar;
  dfduu *= scalar;
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation& ScalarFunctionQuadraticApproximation::resize(size_t nx, size_t nu) {
  dfdx.resize(nx);
  dfdu.resize(nu);
  dfdxx.resize(nx, nx);
  dfdux.resize(nu, nx);
  dfduu.resize(nu, nu);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation& ScalarFunctionQuadraticApproximation::setZero(size_t nx, size_t nu) {
  f = 0.0;
  dfdx.setZero(nx);
  dfdu.setZero(nu);
  dfdxx.setZero(nx, nx);
  dfdux.setZero(nu, nx);
  dfduu.setZero(nu, nu);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ScalarFunctionQuadraticApproximation ScalarFunctionQuadraticApproximation::Zero(size_t nx, size_t nu) {
  ScalarFunctionQuadraticApproximation f;
  f.setZero(nx, nu);
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::string checkBeingPSD(const ScalarFunctionQuadraticApproximation& data, const std::string& dataName) {
  const bool checkState = (data.dfdx.size() > 0);
  const bool checkInput = (data.dfdu.size() > 0);

  std::stringstream errorDescription;

  // check if they are valid values
  if (data.f != data.f) {
    errorDescription << dataName << " is not finite.\n";
  }
  if (checkState && !data.dfdx.allFinite()) {
    errorDescription << dataName << " first derivative w.r.t. state is not finite.\n";
  }
  if (checkState && !data.dfdxx.allFinite()) {
    errorDescription << dataName << " second derivative w.r.t. state is not finite.\n";
  }
  if (checkInput && !data.dfdu.allFinite()) {
    errorDescription << dataName << " first derivative w.r.t. input is not finite.\n";
  }
  if (checkInput && !data.dfduu.allFinite()) {
    errorDescription << dataName << " second derivative w.r.t. input is not finite.\n";
  }
  if (checkState && checkInput && !data.dfdux.allFinite()) {
    errorDescription << dataName << " second derivative w.r.t. input-state is not finite.\n";
  }

  // check for being self-adjoint
  if (checkState && !data.dfdxx.isApprox(data.dfdxx.transpose())) {
    errorDescription << dataName << " second derivative w.r.t. state is not self-adjoint.\n";
  }
  if (checkInput && !data.dfduu.isApprox(data.dfduu.transpose())) {
    errorDescription << dataName << " second derivative w.r.t. input is not self-adjoint.\n";
  }

  // check for being psd
  if (checkState) {
    const auto stateHessianMinEigenvalue = LinearAlgebra::symmetricEigenvalues(data.dfdxx).minCoeff();
    if (stateHessianMinEigenvalue < -Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription << dataName
                       << " second derivative w.r.t. state is not PSD. It's smallest eigenvalue is " +
                              std::to_string(stateHessianMinEigenvalue) + ".\n";
    }
  }
  if (checkInput) {
    const auto inputHessianMinEigenvalue = LinearAlgebra::symmetricEigenvalues(data.dfduu).minCoeff();
    if (inputHessianMinEigenvalue < -Eigen::NumTraits<scalar_t>::epsilon()) {
      errorDescription << dataName
                       << " second derivative w.r.t. input is not PSD. It's smallest eigenvalue is " +
                              std::to_string(inputHessianMinEigenvalue) + ".\n";
    }
  }

  return errorDescription.str();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const ScalarFunctionQuadraticApproximation& f) {
  out << "f: " << f.f << '\n';
  out << "dfdx: " << f.dfdx.transpose() << '\n';
  out << "dfdu: " << f.dfdu.transpose() << '\n';
  out << "dfdxx:\n" << f.dfdxx << '\n';
  out << "dfdux:\n" << f.dfdux << '\n';
  out << "dfduu:\n" << f.dfduu << '\n';
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation::VectorFunctionLinearApproximation(size_t nv, size_t nx, size_t nu) {
  resize(nv, nx, nu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation& VectorFunctionLinearApproximation::resize(size_t nv, size_t nx, size_t nu) {
  f.resize(nv);
  dfdx.resize(nv, nx);
  dfdu.resize(nv, nu);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation& VectorFunctionLinearApproximation::setZero(size_t nv, size_t nx, size_t nu) {
  f.setZero(nv);
  dfdx.setZero(nv, nx);
  dfdu.setZero(nv, nu);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation VectorFunctionLinearApproximation::Zero(size_t nv, size_t nx, size_t nu) {
  VectorFunctionLinearApproximation f;
  f.setZero(nv, nx, nu);
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const VectorFunctionLinearApproximation& f) {
  out << "f: " << f.f.transpose() << '\n';
  out << "dfdx:\n" << f.dfdx << '\n';
  out << "dfdu:\n" << f.dfdu << '\n';
  return out;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation::VectorFunctionQuadraticApproximation(size_t nv, size_t nx, size_t nu) {
  resize(nv, nx, nu);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation& VectorFunctionQuadraticApproximation::resize(size_t nv, size_t nx, size_t nu) {
  f.resize(nv);
  dfdx.resize(nv, nx);
  dfdu.resize(nv, nu);
  dfdxx.resize(nv);
  dfdux.resize(nv);
  dfduu.resize(nv);
  for (size_t i = 0; i < nv; i++) {
    dfdxx[i].resize(nx, nx);
    dfdux[i].resize(nu, nx);
    dfduu[i].resize(nu, nu);
  }
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation& VectorFunctionQuadraticApproximation::setZero(size_t nv, size_t nx, size_t nu) {
  f.setZero(nv);
  dfdx.setZero(nv, nx);
  dfdu.setZero(nv, nu);
  dfdxx.resize(nv);
  dfdux.resize(nv);
  dfduu.resize(nv);
  for (size_t i = 0; i < nv; i++) {
    dfdxx[i].setZero(nx, nx);
    dfdux[i].setZero(nu, nx);
    dfduu[i].setZero(nu, nu);
  }
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionQuadraticApproximation VectorFunctionQuadraticApproximation::Zero(size_t nv, size_t nx, size_t nu) {
  VectorFunctionQuadraticApproximation f;
  f.setZero(nv, nx, nu);
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const VectorFunctionQuadraticApproximation& f) {
  out << "f: " << f.f.transpose() << '\n';
  out << "dfdx:\n" << f.dfdx << '\n';
  out << "dfdu:\n" << f.dfdu << '\n';
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfdxx[" << i << "]:\n" << f.dfdxx[i] << '\n';
  }
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfdux[" << i << "]:\n" << f.dfdux[i] << '\n';
  }
  for (size_t i = 0; i < f.f.rows(); i++) {
    out << "dfduu[" << i << "]:\n" << f.dfduu[i] << '\n';
  }
  return out;
}

}  // namespace ocs2
