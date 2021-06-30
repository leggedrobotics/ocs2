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

#include <cmath>

#include <ocs2_core/dynamics/ControlledSystemBase.h>

namespace ocs2 {

class pendulum_dyn final : public ControlledSystemBase {
 public:
  pendulum_dyn() = default;
  ~pendulum_dyn() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    const double g = 9.81;
    const double L = 1;
    vector_t dxdt(2);
    dxdt << x[1], -(g / L) * std::sin(x[0]);
    return dxdt;
  }

  vector_t computeJumpMap(scalar_t t, const vector_t& state, const PreComputation&) override {
    vector_t mappedState(2);
    mappedState[0] = state[0];
    mappedState[1] = -0.9 * state[1];
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t t, const vector_t& state) override {
    vector_t guardSurfaces(1);
    guardSurfaces[0] = state[0];
    return guardSurfaces;
  }

  pendulum_dyn* clone() const override { return new pendulum_dyn(*this); }
};

}  // namespace ocs2
