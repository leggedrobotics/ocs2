/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <cmath>
#include <ostream>

#include <ocs2_core/Types.h>

namespace ocs2 {

/** Check for approximate equality of ScalarFunctionQuadraticApproximation */
inline bool isApprox(const ScalarFunctionQuadraticApproximation& a, const ScalarFunctionQuadraticApproximation& b,
                     scalar_t precision = 1e-9) {
  return std::abs(a.f - b.f) < precision && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision) &&
         a.dfdxx.isApprox(b.dfdxx, precision) && a.dfduu.isApprox(b.dfduu, precision) && a.dfdux.isApprox(b.dfdux, precision);
}

/** Check for approximate equality of VectorFunctionLinearApproximation */
inline bool isApprox(const VectorFunctionLinearApproximation& a, const VectorFunctionLinearApproximation& b, scalar_t precision = 1e-9) {
  return a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision);
}

/** Check for approximate equality of VectorFunctionQuadraticApproximation */
inline bool isApprox(const VectorFunctionQuadraticApproximation& a, const VectorFunctionQuadraticApproximation& b,
                     scalar_t precision = 1e-9) {
  if (!(a.f.isApprox(b.f, precision) && a.dfdx.isApprox(b.dfdx, precision) && a.dfdu.isApprox(b.dfdu, precision))) {
    return false;
  }
  for (size_t i = 0; i < a.f.rows(); i++) {
    if (!(a.dfdxx[i].isApprox(b.dfdxx[i], precision) && a.dfdux[i].isApprox(b.dfdux[i], precision) &&
          a.dfduu[i].isApprox(b.dfduu[i], precision))) {
      return false;
    }
  }
  return true;
}

/**
 * Compares two Eigen vectors on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const vector_t& lhs, const vector_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  if (lhs.norm() > tol && rhs.norm() > tol) {
    return lhs.isApprox(rhs, tol);
  } else {
    return (lhs - rhs).norm() < tol;
  }
}

/**
 * Compares two scalars on equality in way that is consistent with the check for vectors.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const scalar_t& lhs, const scalar_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return isEqual((vector_t(1) << lhs).finished(), (vector_t(1) << rhs).finished(), tol);
}

/**
 * Compares two Eigen matrices on equality.
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 */
inline bool isEqual(const matrix_t& lhs, const matrix_t& rhs, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  if (lhs.norm() > tol && rhs.norm() > tol) {
    return lhs.isApprox(rhs, tol);
  } else {
    return (lhs - rhs).norm() < tol;
  }
}

/**
 * Compares two trajectories on element-wise approximate equality
 * @param tol : tolerance (default value is 1e-12, which is the default of isApprox().
 * @return Vectors are of equal length and equal values.
 */
template <typename T>
inline bool isEqual(const std::vector<T>& v0, const std::vector<T>& v1, scalar_t tol = Eigen::NumTraits<scalar_t>::dummy_precision()) {
  return (v0.size() == v1.size()) &&
         std::equal(v0.begin(), v0.end(), v1.begin(), [tol](const T& lhs, const T& rhs) { return isEqual(lhs, rhs, tol); });
}

}  // namespace ocs2
