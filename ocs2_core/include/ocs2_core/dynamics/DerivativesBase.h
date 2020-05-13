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

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Base class for the linearized system dynamics. \n
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 */
class DerivativesBase {
 public:
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
  virtual void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u);

  /**
   * Get partial time derivative of the system flow map.
   * \f$ \frac{\partial f}{\partial t}  \f$.
   *
   * @param [out] df: \f$ \frac{\partial f}{\partial t} \f$ matrix, size \f$ n_x \f$.
   */
  virtual void getFlowMapDerivativeTime(vector_t& df);

  /**
   * Get the A matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] A: \f$ A(t) \f$ matrix, size \f$ n_x * n_x \f$.
   */
  virtual void getFlowMapDerivativeState(matrix_t& A) = 0;

  /**
   * Get the B matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] B: \f$ B(t) \f$ matrix, size \f$ n_x * n_u \f$.
   */
  virtual void getFlowMapDerivativeInput(matrix_t& B) = 0;

  /**
   * Get partial time derivative of the system jump map.
   * \f$ \frac{\partial g}{\partial t}  \f$.
   *
   * @param [out] dg: \f$ \frac{\partial g}{\partial t} \f$ matrix.
   */
  virtual void getJumpMapDerivativeTime(vector_t& dg);

  /**
   * Get the G matrix at a given operating point for the linearized system jump map.
   * \f$ x^+ = G \delta x + H \delta u \f$.
   *
   * @param [out] G: \f$ G \f$ matrix, size \f$ n_x * n_x \f$.
   */
  virtual void getJumpMapDerivativeState(matrix_t& G);

  /**
   * Get the G matrix at a given operating point for the linearized system jump map.
   * \f$ x^+ = G \delta x + H \delta u \f$.
   *
   * @param [out] H: \f$ H \f$ matrix, size \f$ n_x * n_u \f$.
   */
  virtual void getJumpMapDerivativeInput(matrix_t& H);

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_t_gamma: Derivative of the guard surfaces w.r.t. time.
   */
  virtual void getGuardSurfacesDerivativeTime(vector_t& D_t_gamma);

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
   */
  virtual void getGuardSurfacesDerivativeState(matrix_t& D_x_gamma);

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @param [out] D_x_gamma: Derivative of the guard surfaces w.r.t. state vector.
   */
  virtual void getGuardSurfacesDerivativeInput(matrix_t& D_u_gamma);

  /**
   * Get at a given operating point the covariance of the dynamics.
   *
   * @param [out] dynamicsCovariance: The covariance of the dynamics.
   */
  virtual void getDynamicsCovariance(matrix_t& dynamicsCovariance);

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual DerivativesBase* clone() const = 0;

 protected:
  scalar_t t_;
  vector_t x_;
  vector_t u_;
};

}  // namespace ocs2
