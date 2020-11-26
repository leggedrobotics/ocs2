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

#include <ocs2_core/Types.h>

namespace ocs2 {

enum class ConstraintOrder { Linear, Quadratic };

/**
 *   Implements the cost penalty for the inequality constraint
 *   \f$ h_i(x, u) \geq 0 \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty = \f$ \sum_{i=1}^{M} p(h_i(x, u)) \f$
 *
 *   The scalar penalty function \f$ p() \f$ and its derivatives are to be implemented in the derived class.
 *   This base class implements the chain rule from the inequality constraint and the implemented penalty function.
 */
class SoftConstraintPenaltyBase {
 public:
  /** Default constructor */
  SoftConstraintPenaltyBase() = default;

  /** Default destructor */
  virtual ~SoftConstraintPenaltyBase() = default;

  /**
   * Get the penalty cost.
   *
   * @param [in] h: Vector of inequality constraint values
   * @return Penalty: The penalty cost.
   */
  scalar_t getValue(const vector_t& h) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: The constraint linear approximation.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const VectorFunctionLinearApproximation& h) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] h: The constraint quadratic approximation.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const VectorFunctionQuadraticApproximation& h) const;

 private:
  /**
   * Compute the penalty value at a certain constraint value.
   *
   * @param [in] i: The index of the constraint in the constraint vector.
   * @param [in] h: Constraint value.
   * @return penalty cost.
   */
  virtual scalar_t getPenaltyValue(size_t i, scalar_t h) const = 0;

  /**
   * Compute the penalty derivative at a certain constraint value.
   *
   * @param [in] i: The index of the constraint in the constraint vector.
   * @param [in] h: Constraint value.
   * @return penalty derivative with respect to constraint value.
   */
  virtual scalar_t getPenaltyDerivative(size_t i, scalar_t h) const = 0;

  /**
   * Compute the penalty second derivative at a certain constraint value.
   *
   * @param [in] i: The index of the constraint in the constraint vector.
   * @param [in] h: Constraint value.
   * @return penalty second derivative with respect to constraint value.
   */
  virtual scalar_t getPenaltySecondDerivative(size_t i, scalar_t h) const = 0;
};

}  // namespace ocs2
