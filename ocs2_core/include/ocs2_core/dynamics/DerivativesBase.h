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

#ifndef DERIVATIVESBASE_OCS2_H_
#define DERIVATIVESBASE_OCS2_H_

#include <cstring>
#include <memory>

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
 * Base class for the linearized system dynamics. \n
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class DerivativesBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const DerivativesBase<STATE_DIM, INPUT_DIM> >;

  using DIMENSIONS = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;
  using state_matrix_t = typename DIMENSIONS::state_matrix_t;
  using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;
  using constraint1_state_matrix_t = typename DIMENSIONS::constraint1_state_matrix_t;
  using constraint1_input_matrix_t = typename DIMENSIONS::constraint1_input_matrix_t;
  using constraint2_state_matrix_t = typename DIMENSIONS::constraint2_state_matrix_t;
  using dynamic_vector_t = typename DIMENSIONS::dynamic_vector_t;
  using dynamic_state_matrix_t = typename DIMENSIONS::dynamic_state_matrix_t;
  using dynamic_input_matrix_t = typename DIMENSIONS::dynamic_input_matrix_t;

  /**
   * Default constructor
   */
  DerivativesBase() = default;

  /**
   * Default destructor
   */
  virtual ~DerivativesBase() = default;

  /**
   * Sets the current time, state, and control inout.
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
   * Get partial time derivative of the system flow map.
   * \f$ \frac{\partial f}{\partial t}  \f$.
   *
   * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix.
   */
  virtual void getFlowMapDerivativeTime(state_vector_t& df) { df.setZero(); }

  /**
   * Get the A matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] A: \f$ A(t) \f$ matrix.
   */
  virtual void getFlowMapDerivativeState(state_matrix_t& A) = 0;

  /**
   * Get the B matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] B: \f$ B(t) \f$ matrix.
   */
  virtual void getFlowMapDerivativeInput(state_input_matrix_t& B) = 0;

  /**
   * Get partial time derivative of the system jump map.
   * \f$ \frac{\partial g}{\partial t}  \f$.
   *
   * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
   */
  virtual void getJumpMapDerivativeTime(state_vector_t& dg) { dg.setZero(); }

  /**
   * Get the G matrix at a given operating point for the linearized system jump map.
   * \f$ x^+ = G \delta x + H \delta u \f$.
   *
   * @param [out] G: \f$ G \f$ matrix.
   */
  virtual void getJumpMapDerivativeState(state_matrix_t& G) { G.setIdentity(); }

  /**
   * Get the G matrix at a given operating point for the linearized system jump map.
   * \f$ x^+ = G \delta x + H \delta u \f$.
   *
   * @param [out] G: \f$ G \f$ matrix.
   */
  virtual void getJumpMapDerivativeInput(state_input_matrix_t& H) { H.setIdentity(); }

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_t_gamma: Derivative of the guard surfaces w.r.t. time.
   */
  virtual void getGuardSurfacesDerivativeTime(dynamic_vector_t& D_t_gamma) { D_t_gamma = dynamic_vector_t::Zero(1); }

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
   */
  virtual void getGuardSurfacesDerivativeState(dynamic_state_matrix_t& D_x_gamma) {
    D_x_gamma = dynamic_state_matrix_t::Zero(1, STATE_DIM);
  }

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
   */
  virtual void getGuardSurfacesDerivativeInput(dynamic_input_matrix_t& D_u_gamma) {
    D_u_gamma = dynamic_input_matrix_t::Zero(1, INPUT_DIM);
  }

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual DerivativesBase<STATE_DIM, INPUT_DIM>* clone() const = 0;

 protected:
  scalar_t t_;
  state_vector_t x_;
  input_vector_t u_;
};

}  // namespace ocs2

#endif /* DERIVATIVESBASE_OCS2_H_ */
