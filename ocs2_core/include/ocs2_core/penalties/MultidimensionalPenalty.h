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

#include <memory>

#include <ocs2_core/Types.h>

#include <ocs2_core/penalties/penalties/PenaltyBase.h>
#include "ocs2_core/penalties/augmented/AugmentedPenaltyBase.h"

namespace ocs2 {

/**
 *   A helper class that implements the penalty for multidimensional constraint
 *   \f$ h_i(x, u) \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty(t, x, u) = \f$ \sum_{i=1}^{M} p(t, h_i(x, u)) \f$
 *
 *   This class uses the chain rule to compute the second-order approximation of the constraint-penalty. In the case that the
 *   second-order approximation of constraint is not provided, it employs a Gauss-Newton approximation technique which only
 *   relies on the first-order approximation. In general, the penalty function can be a function of time.
 */
class MultidimensionalPenalty final {
 public:
  /**
   * Constructor
   * @note This imposes a fixed number of constraints, where the corresponding penalty function in the array is applied.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  template <class PenaltyType>
  MultidimensionalPenalty(std::vector<std::unique_ptr<PenaltyType>> penaltyPtrArray);

  /**
   * Constructor with s single penalty function
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
   */
  template <class PenaltyType>
  MultidimensionalPenalty(std::unique_ptr<PenaltyType> penaltyPtr);

  /** Default destructor */
  ~MultidimensionalPenalty() = default;

  /** Copy constructor */
  MultidimensionalPenalty(const MultidimensionalPenalty& other);

  /**
   * Get the penalty cost.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: Vector of inequality constraint values.
   * @return Penalty: The penalty cost.
   */
  scalar_t getValue(scalar_t t, const vector_t& h, const vector_t* l = nullptr) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: The constraint linear approximation.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const VectorFunctionLinearApproximation& h,
                                                                 const vector_t* l = nullptr) const;

  /**
   * Get the derivative of the penalty cost.
   * Implements the chain rule between the inequality constraint and penalty function.
   *
   * @param [in] t: The time that the constraint is evaluated.
   * @param [in] h: The constraint quadratic approximation.
   * @return The penalty cost quadratic approximation.
   */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const VectorFunctionQuadraticApproximation& h,
                                                                 const vector_t* l = nullptr) const;

  /**
   * Updates the Lagrange multipliers.
   *
   * @param [in] t: The time stamp.
   * @param [in] l: The Lagrange multipliers.
   * @param [in] h: The vector of constraint values.
   * @return updated Lagrange multipliers.
   */
  vector_t updateMultipliers(scalar_t t, const vector_t& h, const vector_t& l) const;

  /**
   * Initializes the Lagrange multipliers.
   *
   * @param [in] numConstraints: Number of constraints associated to this penalty.
   * @return Initial Lagrange multipliers.
   */
  vector_t initializeMultipliers(size_t numConstraints) const;

 private:
  std::tuple<scalar_t, vector_t, vector_t> getPenaltyValue1stDev2ndDev(scalar_t t, const vector_t& h, const vector_t* l) const;

  std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray_;
};

}  // namespace ocs2
