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
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

#include "ocs2_ddp/test/bouncingmass/OverallReference.h"

static constexpr size_t STATE_DIM = 3;
static constexpr size_t INPUT_DIM = 1;

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;

class BouncingMassDynamics final : public ocs2::LinearSystemDynamics {
 public:
  BouncingMassDynamics() : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM)) {
    LinearSystemDynamics::A_ << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;

    const scalar_t e = 0.95;
    delta_ << 0.0, 0.0, 0.0, 0.0, -(1.0 + e), 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::G_ = matrix_t::Identity(STATE_DIM, STATE_DIM) + delta_;
  }
  ~BouncingMassDynamics() = default;

  vector_t computeJumpMap(scalar_t t, const vector_t& x) override {
    vector_t mappedState = x + delta_ * x;

    if (x[2] < 5) {
      mappedState[2] += 1;
    }
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t t, const vector_t& x) {
    vector_t guardSurfaces(1);
    guardSurfaces[0] = x[0];
    return guardSurfaces;
  }

  BouncingMassDynamics* clone() const final { return new BouncingMassDynamics(*this); }

 private:
  matrix_t delta_{STATE_DIM, STATE_DIM};
};

class BouncingMassCost final : public ocs2::QuadraticCostFunction {
 public:
  using BASE = ocs2::QuadraticCostFunction;

  BouncingMassCost(OverallReference ref, matrix_t Q, matrix_t R, matrix_t P, vector_t xNom, vector_t uNom, vector_t xFin,
                   scalar_t timeFinal)
      : QuadraticCostFunction(Q, R, xNom, uNom, P, xFin), ref_(ref), timeFinal_(timeFinal) {}

  ~BouncingMassCost() = default;

  BouncingMassCost* clone() const final { return new BouncingMassCost(*this); }

  std::pair<vector_t, vector_t> getNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u) final {
    vector_t xRef;
    vector_t uRef;
    const int currentMode = x.tail(1).value();
    ref_.getInput(t, uRef);
    ref_.getState(currentMode, t, xRef);
    return {xRef, uRef};
  }

  vector_t getNominalFinalState(scalar_t t, const vector_t& x) final {
    // Terminal cost only calculated for final state, not for intermediate switch states
    if (std::fabs(t - timeFinal_) > ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon()) {
      return x;
    }

    vector_t xRef;
    const int currentMode = x.tail(1).value();
    ref_.getState(currentMode, t, xRef);
    return xRef;
  }

 private:
  scalar_t timeFinal_;

  OverallReference ref_;
};
