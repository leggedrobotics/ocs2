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
  /**
   * Constructor
   *
   * @param [in] stateDim: State vector dimension
   * @param [in] inputDim: Input vector dimension
   */
  ConstraintBase(size_t stateDim, size_t inputDim);

  /**
   * Default copy constructor
   */
  ConstraintBase(const ConstraintBase& rhs) = default;

  /**
   * Default destructor
   */
  virtual ~ConstraintBase() = default;

  /**
   * Sets the current time, state, and control input.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   */
  virtual void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Clones the class.
   *
   * @return A raw pointer to the class.
   */
  virtual ConstraintBase* clone() const;

  /**
   * Computes the state-input equality constraints.
   *
   * @return The state-input equality constraints value.
   */
  virtual vector_t getStateInputEqualityConstraint();

  /**
   * Gets the state-only equality constraints.
   *
   * @return The state-only equality constraints value.
   */
  virtual vector_t getStateEqualityConstraint();

  /**
   * Gets the inequality constraints.
   *
   *  \f$ h(x, u, t) \geq 0 \f$
   *
   * @return Vector of inequality constraints values.
   */
  virtual scalar_array_t getInequalityConstraint();

  /**
   * Compute the final state-only equality constraints.
   *
   * @return The final state-only equality constraints value.
   */
  virtual vector_t getFinalStateEqualityConstraint();

  /**
   * The C matrix at a given operating point for the linearized state-input constraints,
   * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
   *
   * @return \f$ C(t) \f$ matrix.
   */
  virtual matrix_t getStateInputEqualityConstraintDerivativesState();

  /**
   * The D matrix at a given operating point for the linearized state-input constraints,
   * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
   *
   * @return \f$ D(t) \f$ matrix.
   */
  virtual matrix_t getStateInputEqualityConstraintDerivativesInput();

  /**
   * calculate and retrieve the the derivative of the state-input constraints w.r.t. event times.
   * g1DevArray[i] is the partial derivative of state-input equality constraints with respect to i'th event time.
   *
   * If the constraints are not a function of event times either set the array size to zero (as the default)
   * or set it to an array of zero vectors with a size equal to number event times.
   *
   * @return array of vectors.
   */
  virtual vector_array_t getStateInputEqualityConstraintDerivativesEventTimes();

  /**
   * The F matrix at a given operating point for the linearized state-only constraints,
   * \f$ F(t) \delta x + h(t) = 0 \f$.
   *
   * @return \f$ F(t) \f$ matrix.
   */
  virtual matrix_t getStateEqualityConstraintDerivativesState();

  /**
   * Get the derivative of the inequality constraints.
   * @return Vector of derivatives for each constraint with respect to state vector.
   */
  virtual vector_array_t getInequalityConstraintDerivativesState();

  /**
   * Get the derivative of the inequality constraints.
   * @return Vector derivatives for each constraint with respect to input vector.
   */
  virtual vector_array_t getInequalityConstraintDerivativesInput();

  /**
   * Get the second derivative of the inequality constraints.
   * @return Vector of second derivatives for each constraint with respect to state vector.
   */
  virtual matrix_array_t getInequalityConstraintSecondDerivativesState();

  /**
   * Get the second derivative of the inequality constraints.
   * @return Vector of second derivatives for each constraint with respect to input vector.
   */
  virtual matrix_array_t getInequalityConstraintSecondDerivativesInput();

  /**
   * Get the second derivative of the inequality constraints.
   * @return Vector of second derivatives for each constraint with respect to input vector and state.
   */
  virtual matrix_array_t getInequalityConstraintDerivativesInputState();

  /**
   * The F matrix at a given operating point for the linearized terminal state-only constraints,
   * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
   *
   * @return \f$ F_f(t) \f$ matrix.
   */
  virtual matrix_t getFinalStateEqualityConstraintDerivativesState();

 protected:
  size_t stateDim_;
  size_t inputDim_;
  scalar_t t_;
  vector_t x_;
  vector_t u_;
};

}  // end of namespace ocs2
