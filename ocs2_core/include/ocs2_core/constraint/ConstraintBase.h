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

#ifndef CONSTRAINTBASE_OCS2_H_
#define CONSTRAINTBASE_OCS2_H_

#include <cstring>
#include <memory>

#include "ocs2_core/Dimensions.h"

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
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class ConstraintBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<ConstraintBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const ConstraintBase<STATE_DIM, INPUT_DIM> >;

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
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using input_state_matrix_t = typename DIMENSIONS::input_state_matrix_t;
  using input_state_matrix_array_t = typename DIMENSIONS::input_state_matrix_array_t;
  using constraint1_vector_t = typename DIMENSIONS::constraint1_vector_t;
  using constraint1_vector_array_t = typename DIMENSIONS::constraint1_vector_array_t;
  using constraint2_vector_t = typename DIMENSIONS::constraint2_vector_t;
  using constraint2_vector_array_t = typename DIMENSIONS::constraint2_vector_array_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
  using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;

  /**
   * Default constructor
   */
  ConstraintBase() = default;

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
  virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
    t_ = t;
    x_ = x;
    u_ = u;
  }

  /**
   * Clones the class.
   *
   * @return A raw pointer to the class.
   */
  virtual ConstraintBase<STATE_DIM, INPUT_DIM>* clone() const { return new ConstraintBase<STATE_DIM, INPUT_DIM>(*this); }

  /**
   * Computes the state-input equality constraints.
   *
   * @param [out] e: The state-input equality constraints value.
   */
  virtual void getConstraint1(constraint1_vector_t& e) {}

  /**
   * Gets the number of active state-input equality constraints.
   *
   * @param [in] time: time.
   * @return number of state-input active equality constraints.
   */
  virtual size_t numStateInputConstraint(const scalar_t& time) { return 0; }

  /**
   * Gets the state-only equality constraints.
   *
   * @param [out] h: The state-only (in)equality constraints value.
   */
  virtual void getConstraint2(constraint2_vector_t& h) {}

  /**
   * Get the number of state-only active equality constraints.
   *
   * @param [in] time: time.
   * @return number of state-only active (in)equality constraints.
   */
  virtual size_t numStateOnlyConstraint(const scalar_t& time) { return 0; }

  /**
   * Gets the inequality constraints.
   *
   *  \f$ h(x, u, t) \geq 0 \f$
   *
   * @param [out] h: Vector of inequality constraints values.
   */
  virtual void getInequalityConstraint(scalar_array_t& h) {}

  /**
   * Get the number of inequality constraints.
   *
   * @param [in] time: time.
   * @return number of inequality constraints.
   */
  virtual size_t numInequalityConstraint(const scalar_t& time) { return 0; }

  /**
   * Compute the final state-only equality constraints.
   *
   * @param [out] h_f: The final state-only (in)equality constraints value.
   */
  virtual void getFinalConstraint2(constraint2_vector_t& h_f) {}

  /**
   * Get the number of final state-only active (in)equality constraints.
   *
   * @param [in] time: time.
   * @return number of final state-only active equality constraints.
   */
  virtual size_t numStateOnlyFinalConstraint(const scalar_t& time) { return 0; }

  /**
   * The C matrix at a given operating point for the linearized state-input constraints,
   * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
   *
   * @param [out] C: \f$ C(t) \f$ matrix.
   */
  virtual void getConstraint1DerivativesState(constraint1_state_matrix_t& C) {}

  /**
   * The D matrix at a given operating point for the linearized state-input constraints,
   * \f$ C(t) \delta x + D(t) \delta u + e(t) = 0 \f$.
   *
   * @param [out] D: \f$ D(t) \f$ matrix.
   */
  virtual void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) {}

  /**
   * calculate and retrieve the the derivative of the state-input constraints w.r.t. event times.
   * g1DevArray[i] is a vector of dimension MAX_CONSTRAINT1_DIM_ which is the partial derivative of
   * state-input equality constraints with respect to i'th event time.
   *
   * Note that only nc1 top rows of g1DevArray[i] are valid where nc1 is the number of active
   * state-input constraints at the current time.
   *
   * If the constraints are not a function of event times either set the array size to zero (as the default)
   * implementation or set it to an array of zero vector with a size equal to number event times.
   *
   * @param [out] g1DevArray: an array of nc1-by-1 vector.
   */
  virtual void getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) { g1DevArray.clear(); }

  /**
   * The F matrix at a given operating point for the linearized state-only constraints,
   * \f$ F(t) \delta x + h(t) = 0 \f$.
   *
   * @param [out] F: \f$ F(t) \f$ matrix.
   */
  virtual void getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

  /**
   * Get the derivative of the inequality constraints.
   * @param [out] dhdx: Vector of derivatives for each constraint with respect to state vector.
   */
  virtual void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) {}

  /**
   * Get the derivative of the inequality constraints.
   * @param [out] dhdu: Vector derivatives for each constraint with respect to input vector.
   */
  virtual void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) {}

  /**
   * Get the second derivative of the inequality constraints.
   * @param [out] ddhdxdx: Vector of second derivatives for each constraint with respect to state vector.
   */
  virtual void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) {}

  /**
   * Get the second derivative of the inequality constraints.
   * @param [out] ddhudu: Vector of second derivatives for each constraint with respect to input vector.
   */
  virtual void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) {}

  /**
   * Get the second derivative of the inequality constraints.
   * @param [out] ddhudx: Vector of second derivatives for each constraint with respect to input vector and state.
   */
  virtual void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) {}

  /**
   * The F matrix at a given operating point for the linearized terminal state-only constraints,
   * \f$ F_f(t) \delta x + h_f(t) = 0 \f$.
   *
   * @param [out] F_f: \f$ F_f(t) \f$ matrix.
   */
  virtual void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_f) {}

 protected:
  scalar_t t_;
  state_vector_t x_;
  input_vector_t u_;
};

}  // end of namespace ocs2

#endif /* CONSTRAINTBASE_OCS2_H_ */
