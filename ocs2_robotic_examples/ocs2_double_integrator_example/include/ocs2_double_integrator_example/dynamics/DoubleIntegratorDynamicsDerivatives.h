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

#ifndef DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_OCS2_H_
#define DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_OCS2_H_

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "ocs2_double_integrator_example/definitions.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorDynamicsDerivatives : public DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<DoubleIntegratorDynamicsDerivatives> Ptr;
  typedef std::shared_ptr<const DoubleIntegratorDynamicsDerivatives> ConstPtr;

  typedef DerivativesBase<double_integrator::STATE_DIM_, double_integrator::INPUT_DIM_> BASE;
  typedef typename BASE::scalar_t scalar_t;
  typedef typename BASE::state_vector_t state_vector_t;
  typedef typename BASE::state_matrix_t state_matrix_t;
  typedef typename BASE::input_vector_t input_vector_t;
  typedef typename BASE::state_input_matrix_t state_input_matrix_t;

  /**
   * Constructor
   *
   * @param [in] mass: the inertia of the particle
   */
  DoubleIntegratorDynamicsDerivatives(double mass) {
    A_ << 0.0, 1.0, 0.0, 0.0;
    B_ << 0.0, 1.0 / mass;
  }

  /**
   * Destructor
   */
  ~DoubleIntegratorDynamicsDerivatives() = default;

  /**
   * Returns pointer to the class.
   *
   * @return A raw pointer to the class.
   */
  virtual DoubleIntegratorDynamicsDerivatives* clone() const { return new DoubleIntegratorDynamicsDerivatives(*this); }

  /**
   * Sets the current time, state, and control input.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @param [in] u: Current input.
   */
  virtual void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
    // BASE class method
    BASE::setCurrentStateAndControl(t, x, u);
  }

  /**
   * Get the A matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] A: \f$ A(t) \f$ matrix.
   */
  void getFlowMapDerivativeState(state_matrix_t& A) { A = A_; }

  /**
   * Get the B matrix at a given operating point for the linearized system flow map.
   * \f$ dx/dt = A(t) \delta x + B(t) \delta u \f$.
   *
   * @param [out] B: \f$ B(t) \f$ matrix.
   */
  void getFlowMapDerivativeInput(state_input_matrix_t& B) { B = B_; }

 private:
  state_matrix_t A_;
  state_input_matrix_t B_;
};

}  // namespace double_integrator
}  // namespace ocs2

#endif /* DOUBLE_INTEGRATOR_DYNAMICS_DERIVATIVES_OCS2_H_ */
