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
#include <ocs2_core/dynamics/ControlledSystemBase.h>

namespace ocs2 {

/**
 * The system dynamics and linearization class.
 * The linearized system flow map is defined as: \n
 * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$ \n
 * The linearized system jump map is defined as: \n
 * \f$ x^+ = G \delta x + H \delta u \f$ \n
 */
class SystemDynamicsBase : public ControlledSystemBase {
 public:
  /**
   * Constructor
   *
   * @param [in] preComputation: The (optional) pre-computation module, internally keeps a copy.
   *                             @see PreComputation class documentation.
   */
  explicit SystemDynamicsBase(const PreComputation& preComputation = PreComputation());

  /** Default destructor */
  ~SystemDynamicsBase() override = default;

  /** Clone */
  SystemDynamicsBase* clone() const override = 0;

  /**
   * Computes the linear approximation.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] u: The current input.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The state time derivative linear approximation.
   */
  virtual VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                const PreComputation& preComp) = 0;

  /** Computes the jump map linear approximation.
   *
   * @param [in] t: The current time.
   * @param [in] x: The current state.
   * @param [in] preComp: pre-computation module, safely ignore this parameter if not used.
   *                      @see PreComputation class documentation.
   * @return The linear approximation of the mapped state after transition
   */
  virtual VectorFunctionLinearApproximation jumpMapLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& preComp);

  /** Computes the guard surfaces linear approximation */
  virtual VectorFunctionLinearApproximation guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get partial time derivative of the system flow map.
   * \f$ \frac{\partial f}{\partial t}  \f$.
   *
   * @return \f$ \frac{\partial f}{\partial t} \f$ matrix, size \f$ n_x \f$.
   */
  virtual vector_t flowMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get partial time derivative of the system jump map.
   * \f$ \frac{\partial g}{\partial t}  \f$.
   *
   * @return \f$ \frac{\partial g}{\partial t} \f$ matrix.
   */
  virtual vector_t jumpMapDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get at a given operating point the derivative of the guard surfaces w.r.t. input vector.
   *
   * @return Derivative of the guard surfaces w.r.t. time.
   */
  virtual vector_t guardSurfacesDerivativeTime(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Get at a given operating point the covariance of the dynamics.
   *
   * @return The covariance of the dynamics.
   */
  virtual matrix_t dynamicsCovariance(scalar_t t, const vector_t& x, const vector_t& u);

  /**
   * Computes the flow map linear approximation.
   *
   * @note This method updates the internal preComputation with the request() callback and passes it
   *       to the virtual linearApproximation() with the preComputation parameter.
   *       This interface is used by SensitivityIntegrator.
   */
  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u);

  /** Computes the jump map linear approximation.
   *
   * @note This method updates the internal preComputation with the requestPreJump() callback and
   *       passes it to the virtual jumpMapLinearApproximation() with the preComputation parameter.
   */
  VectorFunctionLinearApproximation jumpMapLinearApproximation(scalar_t t, const vector_t& x);

 protected:
  /** Copy constructor */
  SystemDynamicsBase(const SystemDynamicsBase& other);
};

}  // namespace ocs2
