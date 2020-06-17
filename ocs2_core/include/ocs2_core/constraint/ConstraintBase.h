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

/**
 * Base class for the constraints and its Derivatives. The linearized constraints are defined as: \n
 * Here we consider three types of constraints:
 *
 * - state-input constraints, \f$ g_1(x,u,t) = 0\f$,
 * - state-only constraints, \f$ g_2(x,t) = 0\f$.
 * - inequality constraint, \f$ h(x,u,t) \geq 0\f$
 *
 * \f$ x \f$, \f$ u \f$, and \f$ t \f$ are state, input, and vector of
 * event times respectively.
 *
 * - Linearized state-input constraints:       \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$
 * - Linearized only-state constraints:        \f$ F(t) \delta x + h(t) = 0 \f$
 * - Linearized only-state final constraints:  \f$ F_f(t) \delta x + h_f(t) = 0 \f$
 * - Quadratic approximation of each inequality constraint: \f$ h_{0,i}(t) + h_{x,i}(t) \delta x + h_{u,i}(t) \delta u \f$ \n
 *  \f$ \qquad + 0.5 \delta x  H_{xx,i}(t) \delta x +  \delta u  H_{ux,i}(t) \delta x + 0.5 \delta u  H_{uu,i}(t) \delta u
 *  \geq  0 \quad \forall i \f$
 */
class ConstraintBase {
 public:
  /** Default constructor */
  ConstraintBase() = default;

  /** Default copy constructor */
  ConstraintBase(const ConstraintBase& rhs) = default;

  /** Default destructor */
  virtual ~ConstraintBase() = default;

  /** Clones the class. */
  virtual ConstraintBase* clone() const;

  /**
   * Computes the state-input equality constraints.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   * @return The state-input equality constraints value.
   */
  virtual vector_t stateInputEqualityConstraint(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Gets the state-only equality constraints.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return The state-only equality constraints value.
   */
  virtual vector_t stateEqualityConstraint(scalar_t t, const vector_t& x);

  /**
   * Gets the inequality constraints.
   *
   *  \f$ h(x, u, t) \geq 0 \f$
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   * @return Vector of inequality constraints values.
   */
  virtual vector_t inequalityConstraint(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Compute the final state-only equality constraints.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return The final state-only equality constraints value.
   */
  virtual vector_t finalStateEqualityConstraint(scalar_t t, const vector_t& x);

  /**
   * Gets the state-input equality constraints linear approximation.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   * @return The constraint function linear approximation
   */
  virtual VectorFunctionLinearApproximation stateInputEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x,
                                                                                            const vector_t& u);

  /**
   * Gets the state-only equality constraints linear approximation.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return The constraint function linear approximation (dfdu is not set)
   */
  virtual VectorFunctionLinearApproximation stateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x);

  /**
   * Gets the inequality constraints quadratic approximation.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   * @return The constraint function quadratic approximation
   */
  virtual VectorFunctionQuadraticApproximation inequalityConstraintQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Gets the final state-only equality constraints linear approximation.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return The constraint function linear approximation (dfdu is not set)
   */
  virtual VectorFunctionLinearApproximation finalStateEqualityConstraintLinearApproximation(scalar_t t, const vector_t& x);

  /**
   * calculate and retrieve the the derivative of the state-input constraints w.r.t. event times.
   * g1DevArray[i] is the partial derivative of state-input equality constraints with respect to i'th event time.
   *
   * If the constraints are not a function of event times either set the array size to zero (as the default)
   * or set it to an array of zero vectors with a size equal to number event times.
   *
   * @return array of vectors.
   */
  virtual vector_array_t stateInputEqualityConstraintDerivativesEventTimes(scalar_t t, const vector_t& x, const vector_t& u);
};

}  // end of namespace ocs2
