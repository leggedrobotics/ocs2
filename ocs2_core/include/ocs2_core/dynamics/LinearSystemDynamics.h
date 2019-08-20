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

#ifndef LINEARSYSTEMDYNAMICS_OCS2_H_
#define LINEARSYSTEMDYNAMICS_OCS2_H_

#include "ocs2_core/dynamics/SystemDynamicsBase.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearSystemDynamics : public SystemDynamicsBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<LinearSystemDynamics<STATE_DIM, INPUT_DIM> >;
  using ConstPtr = std::shared_ptr<const LinearSystemDynamics<STATE_DIM, INPUT_DIM> >;

  using BASE = SystemDynamicsBase<STATE_DIM, INPUT_DIM>;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  LinearSystemDynamics(const state_matrix_t& A, const state_input_matrix_t& B, const state_matrix_t& G = state_matrix_t::Zero(),
                       const state_input_matrix_t& H = state_input_matrix_t::Zero())
      : A_(A), B_(B), G_(G), H_(H) {}

  virtual ~LinearSystemDynamics() = default;

  /**
   * Returns pointer to the base class.
   *
   * @return A raw pointer to the class.
   */
  LinearSystemDynamics<STATE_DIM, INPUT_DIM>* clone() const override { return new LinearSystemDynamics<STATE_DIM, INPUT_DIM>(*this); }

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    dxdt = A_ * x + B_ * u;
  }

  void computeJumpMap(const scalar_t& t, const state_vector_t& x, state_vector_t& xp) override { xp = G_ * x; }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override {
    BASE::setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(state_matrix_t& A) override { A = A_; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) override { B = B_; }

  void getJumpMapDerivativeState(state_matrix_t& G) override { G = G_; }

  void getJumpMapDerivativeInput(state_input_matrix_t& H) override { H = H_; }

 private:
  state_matrix_t A_;
  state_input_matrix_t B_;
  state_matrix_t G_;
  state_input_matrix_t H_;
};

}  // namespace ocs2

#endif /* LINEARSYSTEMDYNAMICS_OCS2_H_ */
