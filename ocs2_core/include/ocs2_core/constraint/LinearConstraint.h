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

#include <ocs2_core/constraint/ConstraintBase.h>

namespace ocs2 {

class LinearConstraint final : public ConstraintBase {
 public:
  /**
   * @brief Constructor that sets the number constraints for all types to 0
   *
   * @param[in] stateDim: State vector dimension
   * @param[in] inputDim: Input vector dimension
   */
  LinearConstraint(size_t stateDim, size_t inputDim);

  /**
   * @brief Constructor for only equality constraints
   *
   * @param[in] stateDim: State vector dimension
   * @param[in] inputDim: Input vector dimension
   * @param[in] numStateInputConstraint: Number of state-input equality constraints
   * @param[in] e: Constant term in C * x + D * u + e = 0
   * @param[in] C: x factor in C * x + D * u + e = 0
   * @param[in] D: u factor in C * x + D * u + e = 0
   * @param[in] numStateOnlyConstraint: Number of state-only equality constraints
   * @param[in] h: Constant term in F * x + h = 0
   * @param[in] F: x factor in F * x + h = 0
   * @param[in] numStateOnlyFinalConstraint: Number of final time state-only equality constrains
   * @param[in] h_f: Constant term in F_f * x + h_f = 0 (at final time)
   * @param[in] F_f: x factor in F_f * x + h_f = 0 (at final time)
   */
  LinearConstraint(size_t stateDim, size_t inputDim, size_t numStateInputConstraint, vector_t e, matrix_t C, matrix_t D,
                   size_t numStateOnlyConstraint, vector_t h, matrix_t F, size_t numStateOnlyFinalConstraint, vector_t h_f, matrix_t F_f);

  /**
   * @brief General constructor for equality and inequality constraints
   * @note The inequality constraint can be quadratic of the form
   * h0 + x^T * dhdx + u^T * dhdu + x^T * ddhdxdx * x + u^T * ddhdudu * u + u^T * ddhdudx * x >= 0
   *
   * @param[in] stateDim: State vector dimension
   * @param[in] inputDim: Input vector dimension
   * @param[in] numStateInputConstraint: Number of state-input equality constraints
   * @param[in] e: Constant term in C * x + D * u + e = 0
   * @param[in] C: x factor in C * x + D * u + e = 0
   * @param[in] D: u factor in C * x + D * u + e = 0
   * @param[in] numStateOnlyConstraint: Number of state-only equality constraints
   * @param[in] h: Constant term in F * x + h = 0
   * @param[in] F: x factor in F * x + h = 0
   * @param[in] numStateOnlyFinalConstraint: Number of final time state-only equality constrains
   * @param[in] h_f: Constant term in F_f * x + h_f = 0 (at final time)
   * @param[in] F_f: x factor in F_f * x + h_f = 0 (at final time)
   * @param[in] numInequalityConstraint: Number of inequality constraints
   * @param[in] h0: Constant term in inequality constraint
   * @param[in] dhdx: Linear x multiplier in inequality constraint
   * @param[in] dhdu: Linear u multiplier in inequality constraint
   * @param[in] ddhdxdx: Quadratic x multiplier in inequality constraint
   * @param[in] ddhdudu: Quadratic u multiplier in inequality constraint
   * @param[in] ddhdudx: Quadratic mixed term in inequality constraint
   */
  LinearConstraint(size_t stateDim, size_t inputDim, size_t numStateInputConstraint, vector_t e, matrix_t C, matrix_t D,
                   size_t numStateOnlyConstraint, vector_t h, matrix_t F, size_t numStateOnlyFinalConstraint, vector_t h_f, matrix_t F_f,
                   size_t numInequalityConstraint, scalar_array_t h0, vector_array_t dhdx, vector_array_t dhdu, matrix_array_t ddhdxdx,
                   matrix_array_t ddhdudu, matrix_array_t ddhdudx);

  virtual ~LinearConstraint() override = default;

  LinearConstraint* clone() const override;

  void getConstraint1(vector_t& g1) override;

  size_t numStateInputConstraint(const scalar_t& time) override;

  void getConstraint2(vector_t& g2) override;

  size_t numStateOnlyConstraint(const scalar_t& time) override;

  void getInequalityConstraint(scalar_array_t& h) override;

  size_t numInequalityConstraint(const scalar_t& time) override;

  void getFinalConstraint2(vector_t& g2Final) override;

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override;

  void getConstraint1DerivativesState(matrix_t& C) override;

  void getConstraint1DerivativesControl(matrix_t& D) override;

  void getConstraint2DerivativesState(matrix_t& F) override;

  void getInequalityConstraintDerivativesState(vector_array_t& dhdx) override;
  void getInequalityConstraintDerivativesInput(vector_array_t& dhdu) override;

  void getInequalityConstraintSecondDerivativesState(matrix_array_t& ddhdxdx) override;
  void getInequalityConstraintSecondDerivativesInput(matrix_array_t& ddhdudu) override;
  void getInequalityConstraintDerivativesInputState(matrix_array_t& ddhdudx) override;

  void getFinalConstraint2DerivativesState(matrix_t& F_f) override;

 public:
  size_t numStateInputConstraint_;
  vector_t e_; /**< State input constraint */
  matrix_t C_; /**< State input constraint derivative wrt. state */
  matrix_t D_; /**< State input constraint derivative wrt. input */

  size_t numStateOnlyConstraint_;
  vector_t h_; /**< State only constraint */
  matrix_t F_; /**< State only constraint derivative wrt. state */

  size_t numStateOnlyFinalConstraint_;
  vector_t h_f_; /**< Final state only constraint */
  matrix_t F_f_; /**< Final state only constraint derivative wrt. state */

  size_t numInequalityConstraint_;
  scalar_array_t h0_;      /**< Inequality constraint */
  vector_array_t dhdx_;    /**< Inequality constraint derivative wrt. state */
  vector_array_t dhdu_;    /**< Inequality constraint derivative wrt. input */
  matrix_array_t ddhdxdx_; /**< Inequality constraint second derivative wrt. state */
  matrix_array_t ddhdudu_; /**< Inequality constraint second derivative wrt. input */
  matrix_array_t ddhdudx_; /**< Inequality constraint second derivative wrt. input and state */
};

}  // namespace ocs2
