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

#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <ostream>
#include <vector>

namespace ocs2 {

/** size_t trajectory type. */
using size_array_t = std::vector<size_t>;
/** Array of size_t trajectory type. */
using size_array2_t = std::vector<size_array_t>;

/** Scalar type. */
using scalar_t = double;
/** Scalar trajectory type. */
using scalar_array_t = std::vector<scalar_t>;
/** Array of scalar trajectory type. */
using scalar_array2_t = std::vector<scalar_array_t>;
/** Array of arrays of scalar trajectory type. */
using scalar_array3_t = std::vector<scalar_array2_t>;

/** Dynamic-size vector type. */
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
/** Dynamic vector's trajectory type. */
using vector_array_t = std::vector<vector_t>;
/** Array of dynamic vector's trajectory type. */
using vector_array2_t = std::vector<vector_array_t>;
/** Array of arrays of dynamic vector trajectory type. */
using vector_array3_t = std::vector<vector_array2_t>;

/** Dynamic-size row vector type. */
using row_vector_t = Eigen::Matrix<scalar_t, 1, Eigen::Dynamic>;

/** Dynamic-size matrix type. */
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
/** Dynamic matrix's trajectory type. */
using matrix_array_t = std::vector<matrix_t>;
/** Array of dynamic matrix's trajectory type. */
using matrix_array2_t = std::vector<matrix_array_t>;
/** Array of arrays of dynamic matrix trajectory type. */
using matrix_array3_t = std::vector<matrix_array2_t>;

/**
 * Defines the linear approximation of a scalar function
 * f(x,u) = dfdx' dx + dfdu' du + f
 */
struct ScalarFunctionLinearApproximation {
  /** First derivative w.r.t state */
  vector_t dfdx;
  /** First derivative w.r.t input */
  vector_t dfdu;
  /** Constant term */
  scalar_t f = 0.;

  /** Default constructor */
  ScalarFunctionLinearApproximation() = default;

  /** Construct and resize the members to given size. (Pass nu = -1 for no inputs) */
  explicit ScalarFunctionLinearApproximation(int nx, int nu = -1);

  /** Compound addition assignment operator */
  ScalarFunctionLinearApproximation& operator+=(const ScalarFunctionLinearApproximation& rhs);

  /** Compound scalar multiplication and assignment operator */
  ScalarFunctionLinearApproximation& operator*=(scalar_t scalar);

  /**
   * Resize the members to the given size
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  ScalarFunctionLinearApproximation& resize(int nx, int nu = -1);

  /**
   * Resizes the members to the given size, and sets all coefficients to zero.
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  ScalarFunctionLinearApproximation& setZero(int nx, int nu = -1);

  /**
   * Factory function with zero initialization
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   * @return Zero initialized object of given size.
   */
  static ScalarFunctionLinearApproximation Zero(int nx, int nu = -1);
};

std::ostream& operator<<(std::ostream& out, const ScalarFunctionLinearApproximation& f);

/**
 * Checks the size of the given linear approximation.
 *
 * @param[in] stateDim: Number of states.
 * @param[in] inputDim: Number of inputs.
 * @param[in] data: Given linear approximation.
 * @param[in] dataName: The name of the data which appears in the output error message.
 * @return The description of the error. If there was no error it would be empty;
 */
std::string checkSize(int stateDim, int inputDim, const ScalarFunctionLinearApproximation& data, const std::string& dataName);

inline ScalarFunctionLinearApproximation operator*(ScalarFunctionLinearApproximation lhs, scalar_t scalar) {
  return lhs *= scalar;
}
inline ScalarFunctionLinearApproximation operator*(scalar_t scalar, ScalarFunctionLinearApproximation rhs) {
  return rhs *= scalar;
}

/**
 * Defines the quadratic approximation of a scalar function
 * f(x,u) = 1/2 dx' dfdxx dx + du' dfdux dx + 1/2 du' dfduu du + dfdx' dx + dfdu' du + f
 */
struct ScalarFunctionQuadraticApproximation {
  /** Second derivative w.r.t state */
  matrix_t dfdxx;
  /** Second derivative w.r.t input (lhs) and state (rhs) */
  matrix_t dfdux;
  /** Second derivative w.r.t input */
  matrix_t dfduu;
  /** First derivative w.r.t state */
  vector_t dfdx;
  /** First derivative w.r.t input */
  vector_t dfdu;
  /** Constant term */
  scalar_t f = 0.;

  /** Default constructor */
  ScalarFunctionQuadraticApproximation() = default;

  /** Construct and resize the members to given size. Pass nu = -1 for no inputs */
  explicit ScalarFunctionQuadraticApproximation(int nx, int nu = -1);

  /** Compound addition assignment operator */
  ScalarFunctionQuadraticApproximation& operator+=(const ScalarFunctionQuadraticApproximation& rhs);

  /** Compound scalar multiplication and assignment operator */
  ScalarFunctionQuadraticApproximation& operator*=(scalar_t scalar);

  /**
   * Resize the members to the given size
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  ScalarFunctionQuadraticApproximation& resize(int nx, int nu = -1);

  /**
   * Resizes the members to the given size, and sets all coefficients to zero.
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  ScalarFunctionQuadraticApproximation& setZero(int nx, int nu = -1);

  /**
   * Factory function with zero initialization
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   * @return Zero initialized object of given size.
   */
  static ScalarFunctionQuadraticApproximation Zero(int nx, int nu = -1);
};

std::ostream& operator<<(std::ostream& out, const ScalarFunctionQuadraticApproximation& f);

/**
 * Checks that the given matrix is valid, self-adjoint, and positive semi-definite (PSD).
 * @param[in] data: Given matrix.
 * @param[in] dataName: The name of the data which appears in the output error message.
 * @return The description of the error. If there was no error it would be empty;
 */
std::string checkBeingPSD(const matrix_t& data, const std::string& dataName);

/**
 * Checks that the given quadratic approximation is valid, self-adjoint, and positive semi-definite (PSD).
 * @param[in] data: Given quadratic approximation.
 * @param[in] dataName: The name of the data which appears in the output error message.
 * @return The description of the error. If there was no error it would be empty;
 */
std::string checkBeingPSD(const ScalarFunctionQuadraticApproximation& data, const std::string& dataName);

/**
 * Checks the size of the given quadratic approximation.
 *
 * @param[in] stateDim: Number of states.
 * @param[in] inputDim: Number of inputs.
 * @param[in] data: Given quadratic approximation.
 * @param[in] dataName: The name of the data which appears in the output error message.
 * @return The description of the error. If there was no error it would be empty;
 */
std::string checkSize(int stateDim, int inputDim, const ScalarFunctionQuadraticApproximation& data, const std::string& dataName);

inline ScalarFunctionQuadraticApproximation operator*(ScalarFunctionQuadraticApproximation lhs, scalar_t scalar) {
  return lhs *= scalar;
}
inline ScalarFunctionQuadraticApproximation operator*(scalar_t scalar, ScalarFunctionQuadraticApproximation rhs) {
  return rhs *= scalar;
}

/**
 * Defines the linear model of a vector-valued function
 * f(x,u) = dfdx dx + dfdu du + f
 */
struct VectorFunctionLinearApproximation {
  /** Derivative w.r.t state */
  matrix_t dfdx;
  /** Derivative w.r.t input */
  matrix_t dfdu;
  /** Constant term */
  vector_t f;

  /** Default constructor */
  VectorFunctionLinearApproximation() = default;

  /** Construct and resize the members to given size. (Pass nu = -1 for no inputs) */
  explicit VectorFunctionLinearApproximation(int nv, int nx, int nu = -1);

  /**
   * Resize the members to the given size
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  VectorFunctionLinearApproximation& resize(int nv, int nx, int nu = -1);

  /**
   * Resizes the members to the given size, and sets all coefficients to zero.
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  VectorFunctionLinearApproximation& setZero(int nv, int nx, int nu = -1);

  /**
   * Factory function with zero initialization
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   * @return Zero initialized object of given size.
   */
  static VectorFunctionLinearApproximation Zero(int nv, int nx, int nu = -1);
};

std::ostream& operator<<(std::ostream& out, const VectorFunctionLinearApproximation& f);

/**
 * Checks the size of the given vector-function linear approximation.
 *
 * @param[in] vectorDim: The vector function dimension.
 * @param[in] stateDim: Number of states.
 * @param[in] inputDim: Number of inputs.
 * @param[in] data: Given linear approximation.
 * @param[in] dataName: The name of the data which appears in the output error message.
 * @return The description of the error. If there was no error it would be empty;
 */
std::string checkSize(int vectorDim, int stateDim, int inputDim, const VectorFunctionLinearApproximation& data,
                      const std::string& dataName);

/**
 * Defines quadratic approximation of a vector-valued function
 * f[i](x,u) = 1/2 dx' dfdxx[i] dx + du' dfdux[i] dx + 1/2 du' dfduu[i] du + dfdx[i,:] dx + dfdu[i,:] du + f[i]
 */
struct VectorFunctionQuadraticApproximation {
  /** Second derivative w.r.t state */
  matrix_array_t dfdxx;
  /** Second derivative w.r.t input (lhs) and state (rhs) */
  matrix_array_t dfdux;
  /** Second derivative w.r.t input */
  matrix_array_t dfduu;
  /** First derivative w.r.t state */
  matrix_t dfdx;
  /** First derivative w.r.t input */
  matrix_t dfdu;
  /** Constant term */
  vector_t f;

  /** Default constructor */
  VectorFunctionQuadraticApproximation() = default;

  /** Construct and resize the members to given size. */
  explicit VectorFunctionQuadraticApproximation(int nv, int nx, int nu = -1);

  /**
   * Resize the members to the given size
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  VectorFunctionQuadraticApproximation& resize(int nv, int nx, int nu = -1);

  /**
   * Resizes the members to the given size, and sets all coefficients to zero.
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   */
  VectorFunctionQuadraticApproximation& setZero(int nv, int nx, int nu = -1);

  /**
   * Factory function with zero initialization
   * @param[in] nv Vector dimension
   * @param[in] nx State dimension
   * @param[in] nu Input dimension (Pass nu = -1 for no inputs)
   * @return Zero initialized object of given size.
   */
  static VectorFunctionQuadraticApproximation Zero(int nv, int nx, int nu = -1);
};

std::ostream& operator<<(std::ostream& out, const VectorFunctionQuadraticApproximation& f);

}  // namespace ocs2
