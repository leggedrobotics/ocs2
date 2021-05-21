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

class ballDyn : public ControlledSystemBase {
 public:
  ballDyn() = default;
  ~ballDyn() = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    matrix_t A(2, 2);
    A << 0.0, 1.0, 0.0, 0.0;
    vector_t F(2, 1);
    F << 0.0, -9.81;
    return A * x + F;
  }

  vector_t computeJumpMap(scalar_t t, const vector_t& x, const PreComputation&) override {
    vector_t mappedState(2);
    mappedState[0] = x[0];
    mappedState[1] = -0.95 * x[1];
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t t, const vector_t& x) override {
    vector_t guardSurfaces(2);
    guardSurfaces[0] = x[0];
    guardSurfaces[1] = -x[0] + 0.5;
    return guardSurfaces;
  }

  ballDyn* clone() const override { return new ballDyn(*this); }
};
}  // namespace ocs2
