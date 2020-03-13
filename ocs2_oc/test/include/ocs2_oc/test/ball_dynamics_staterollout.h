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

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>

namespace ocs2 {

class ballDyn : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControlledSystemBase<2, 1>;
  using state_matrix_t = typename BASE::DIMENSIONS::state_matrix_t;
  using state_input_matrix_t = typename BASE::DIMENSIONS::state_input_matrix_t;

  ballDyn() = default;
  ~ballDyn() = default;

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) override {
    state_matrix_t A;
    A << 0.0, 1.0, 0.0, 0.0;
    state_vector_t F;
    F << 0.0, -9.81;

    dxdt = A * x + F;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    mappedState[0] = state[0];
    mappedState[1] = -0.95 * state[1];
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = state[0];
    guardSurfacesValue[1] = -state[0] + 0.5;
  }

  ballDyn* clone() const override { return new ballDyn(*this); }
};
}  // namespace ocs2
