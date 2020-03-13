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

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
 *   Implements the cost penalty for the inequality constraint
 *   \f$ h_i(x, u) \geq 0 \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty = \f$ \sum_{i=1}^{M} p(h_i(x, u)) \f$
 *
 *   The scalar penalty function \f$ p() \f$ and its derivatives are to be implemented in the derived class.
 *   This base class implements the chain rule from the inequality constraint and the implemented penalty function.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class PenaltyBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using input_vector_array_t = typename DIMENSIONS::input_vector_array_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_matrix_array_t = typename DIMENSIONS::state_matrix_array_t;
  using input_matrix_t = typename DIMENSIONS::input_matrix_t;
  using input_matrix_array_t = typename DIMENSIONS::input_matrix_array_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;

  /**
   * Default constructor
   */
  PenaltyBase() = default;

  /**
   * Default destructor
   */
  virtual ~PenaltyBase() = default;

  /**
   * Get the penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values
   * @param [out] penalty: The penalty cost.
   */
  void getPenaltyCost(const scalar_array_t& h, scalar_t& penalty) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [out] penaltyDerivativeState: Derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostDerivativeState(const scalar_array_t& h, const state_vector_array_t& dhdx,
                                     state_vector_t& penaltyDerivativeState) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input vector.
   * @param [out] penaltyDerivativeInput: Derivative of the penalty cost with respect to input vector.
   */
  void getPenaltyCostDerivativeInput(const scalar_array_t& h, const input_vector_array_t& dhdu,
                                     input_vector_t& penaltyDerivativeInput) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] ddhdxdx: Vector of inequality constraint second derivatives with respect to state.
   * @param [out] penaltySecondDerivativeState: Second derivative of the penalty cost with respect to state.
   */
  void getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const state_vector_array_t& dhdx, const state_matrix_array_t& ddhdxdx,
                                           state_matrix_t& penaltySecondDerivativeState) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudu: Vector of inequality constraint second derivatives with respect to input.
   * @param [out] penaltySecondDerivativeInput: Second derivative of the penalty cost with respect to input.
   */
  void getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const input_vector_array_t& dhdu, const input_matrix_array_t& ddhdudu,
                                           input_matrix_t& penaltySecondDerivativeInput) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudx: Vector of inequality constraint derivatives with respect to input and state.
   * @param [out] penaltyDerivativeInputState: Derivative of the penalty cost with respect to input and state.
   */
  void getPenaltyCostDerivativeInputState(const scalar_array_t& h, const state_vector_array_t& dhdx, const input_vector_array_t& dhdu,
                                          const input_state_matrix_array_t& ddhdudx,
                                          input_state_matrix_t& penaltyDerivativeInputState) const;

  /**
   * Computes the sum of squared constraint violation.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [out] squaredViolation: sum of squared constraint violation.
   */
  void getConstraintViolationSquaredNorm(const scalar_array_t& h, scalar_t& squaredViolation) const;

 private:
  /**
   * Compute the penalty value at a certain constraint value.
   *
   * @param [in] h: Constraint value.
   * @return penalty cost.
   */
  virtual scalar_t getPenaltyFunctionValue(scalar_t h) const = 0;

  /**
   * Compute the penalty derivative at a certain constraint value.
   *
   * @param [in] h: Constraint value.
   * @return penalty derivative with respect to constraint value.
   */
  virtual scalar_t getPenaltyFunctionDerivative(scalar_t h) const = 0;

  /**
   * Compute the penalty second derivative at a certain constraint value.
   *
   * @param [in] h: Constraint value.
   * @return penalty second derivative with respect to constraint value.
   */
  virtual scalar_t getPenaltyFunctionSecondDerivative(scalar_t h) const = 0;
};
}  // namespace ocs2

#include "implementation/PenaltyBase.h"
