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

#include "ocs2_core/Types.h"

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
class PenaltyBase {
 public:
  /**
   * Get the penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values
   * @return Penalty: The penalty cost.
   */
  scalar_t getPenaltyCost(const scalar_array_t& h) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @return penaltyDerivativeState: Derivative of the penalty cost with respect to state.
   */
  dynamic_vector_t getPenaltyCostDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input vector.
   * @return penaltyDerivativeInput: Derivative of the penalty cost with respect to input vector.
   */
  dynamic_vector_t getPenaltyCostDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] ddhdxdx: Vector of inequality constraint second derivatives with respect to state.
   * @return penaltySecondDerivativeState: Second derivative of the penalty cost with respect to state.
   */
  dynamic_matrix_t getPenaltyCostSecondDerivativeState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                                       const dynamic_matrix_array_t& ddhdxdx) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudu: Vector of inequality constraint second derivatives with respect to input.
   * @return penaltySecondDerivativeInput: Second derivative of the penalty cost with respect to input.
   */
  dynamic_matrix_t getPenaltyCostSecondDerivativeInput(const scalar_array_t& h, const dynamic_vector_array_t& dhdu,
                                                       const dynamic_matrix_array_t& ddhdudu) const;

  /**
   * Second derivative of penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @param [in] dhdx: Vector of inequality constraint derivatives with respect to state.
   * @param [in] dhdu: Vector of inequality constraint derivatives with respect to input.
   * @param [in] ddhdudx: Vector of inequality constraint derivatives with respect to input and state.
   * @return penaltyDerivativeInputState: Derivative of the penalty cost with respect to input and state.
   */
  dynamic_matrix_t getPenaltyCostDerivativeInputState(const scalar_array_t& h, const dynamic_vector_array_t& dhdx,
                                                      const dynamic_vector_array_t& dhdu, const dynamic_matrix_array_t& ddhdudx) const;

  /**
   * Computes the sum of squared constraint violation.
   *
   * @param [in] h: Vector of inequality constraint values.
   * @return squaredViolation: sum of squared constraint violation.
   */
  scalar_t getConstraintViolationSquaredNorm(const scalar_array_t& h) const;

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
