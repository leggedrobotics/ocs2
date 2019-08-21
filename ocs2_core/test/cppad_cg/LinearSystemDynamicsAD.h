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

#include "ocs2_core/dynamics/SystemDynamicsBaseAD.h"

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class LinearSystemDynamicsAD : public SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = SystemDynamicsBaseAD<STATE_DIM, INPUT_DIM>;
  using typename BASE::ad_dynamic_vector_t;
  using typename BASE::ad_scalar_t;
  using typename BASE::input_vector_t;
  using typename BASE::scalar_t;
  using typename BASE::state_input_matrix_t;
  using typename BASE::state_matrix_t;
  using typename BASE::state_vector_t;

  LinearSystemDynamicsAD(const state_matrix_t& A, const state_input_matrix_t& B, const state_matrix_t& G) : A_(A), B_(B), G_(G) {}

  ~LinearSystemDynamicsAD() = default;

  LinearSystemDynamicsAD(const LinearSystemDynamicsAD& rhs) : BASE(rhs), A_(rhs.A_), B_(rhs.B_), G_(rhs.G_) {}

  LinearSystemDynamicsAD* clone() const override { return new LinearSystemDynamicsAD(*this); }

 protected:
  void systemFlowMap(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                     ad_dynamic_vector_t& stateDerivative) const override {
    stateDerivative = A_.template cast<ad_scalar_t>() * state + B_.template cast<ad_scalar_t>() * input;
  }

  void systemJumpMap(ad_scalar_t time, const ad_dynamic_vector_t& state, ad_dynamic_vector_t& jumpedState) const override {
    jumpedState = G_.template cast<ad_scalar_t>() * state;
  }

 private:
  state_matrix_t A_;
  state_input_matrix_t B_;
  state_matrix_t G_;
};

}  // namespace ocs2
