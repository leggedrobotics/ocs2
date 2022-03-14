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

#include <utility>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>

#include "ocs2_ddp/test/bouncingmass/OverallReference.h"

static constexpr size_t STATE_DIM = 3;
static constexpr size_t INPUT_DIM = 1;

using scalar_t = ocs2::scalar_t;
using vector_t = ocs2::vector_t;
using matrix_t = ocs2::matrix_t;
using PreComputation = ocs2::PreComputation;
using ScalarFunctionQuadraticApproximation = ocs2::ScalarFunctionQuadraticApproximation;

class BouncingMassDynamics final : public ocs2::LinearSystemDynamics {
 public:
  BouncingMassDynamics()
      : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM), matrix_t(STATE_DIM, STATE_DIM)) {
    LinearSystemDynamics::A_ << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;
    const scalar_t e = 0.95;
    LinearSystemDynamics::G_ << 1.0, 0.0, 0.0, 0.0, -e, 0.0, 0.0, 0.0, 1.0;
  }

  ~BouncingMassDynamics() override = default;

  BouncingMassDynamics* clone() const override { return new BouncingMassDynamics(*this); }

  vector_t computeJumpMap(scalar_t t, const vector_t& x, const PreComputation& preComp) override {
    vector_t mappedState = LinearSystemDynamics::computeJumpMap(t, x, preComp);
    if (x(2) < 5) {
      mappedState(2) += 1;
    }
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t t, const vector_t& x) override {
    vector_t guardSurfaces(1);
    guardSurfaces(0) = x(0);
    return guardSurfaces;
  }

  ocs2::VectorFunctionLinearApproximation guardSurfacesLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u) override {
    ocs2::VectorFunctionLinearApproximation approximation;
    approximation.dfdx.setIdentity(1, x.rows());
    approximation.dfdu.setZero(1, u.rows());
    approximation.f = computeGuardSurfaces(t, x);
    return approximation;
  }
};

class BouncingMassCost final : public ocs2::StateInputCost {
 public:
  BouncingMassCost(OverallReference ref, matrix_t Q, matrix_t R) : ref_(ref), Q_(std::move(Q)), R_(std::move(R)) {}

  ~BouncingMassCost() override = default;

  BouncingMassCost* clone() const override { return new BouncingMassCost(*this); }

  scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const ocs2::TargetTrajectories&,
                    const PreComputation&) const override {
    const auto stateInput = getNominalStateInput(t, x, u);
    vector_t xDeviation = x - stateInput.first;
    vector_t uDeviation = u - stateInput.second;
    return 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation);
  }

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const ocs2::TargetTrajectories&, const PreComputation&) const override {
    const auto stateInput = getNominalStateInput(t, x, u);
    vector_t xDeviation = x - stateInput.first;
    vector_t uDeviation = u - stateInput.second;

    ScalarFunctionQuadraticApproximation L;
    L.f = 0.5 * xDeviation.dot(Q_ * xDeviation) + 0.5 * uDeviation.dot(R_ * uDeviation);
    L.dfdx = Q_ * xDeviation;
    L.dfdu = R_ * uDeviation;
    L.dfdxx = Q_;
    L.dfdux.setZero(INPUT_DIM, STATE_DIM);
    L.dfduu = R_;
    return L;
  }

 private:
  std::pair<vector_t, vector_t> getNominalStateInput(scalar_t t, const vector_t& x, const vector_t& u) const {
    vector_t xRef;
    vector_t uRef;
    const int currentMode = x.tail(1).value();
    ref_.getInput(t, uRef);
    ref_.getState(currentMode, t, xRef);
    return {xRef, uRef};
  }

 private:
  OverallReference ref_;
  matrix_t Q_;
  matrix_t R_;
};

class BouncingMassFinalCost final : public ocs2::StateCost {
 public:
  BouncingMassFinalCost(OverallReference ref, matrix_t QFinal, scalar_t timeFinal)
      : ref_(ref), timeFinal_(timeFinal), QFinal_(std::move(QFinal)) {}

  ~BouncingMassFinalCost() override = default;

  BouncingMassFinalCost* clone() const override { return new BouncingMassFinalCost(*this); }

  scalar_t getValue(scalar_t t, const vector_t& x, const ocs2::TargetTrajectories&, const PreComputation&) const override {
    vector_t xDeviation = x - getNominalFinalState(t, x);
    return 0.5 * xDeviation.dot(QFinal_ * xDeviation);
  }

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const ocs2::TargetTrajectories&,
                                                                 const PreComputation&) const override {
    vector_t xDeviation = x - getNominalFinalState(t, x);

    ScalarFunctionQuadraticApproximation Phi;
    Phi.f = 0.5 * xDeviation.dot(QFinal_ * xDeviation);
    Phi.dfdx = QFinal_ * xDeviation;
    Phi.dfdxx = QFinal_;
    return Phi;
  }

 private:
  vector_t getNominalFinalState(scalar_t t, const vector_t& x) const {
    // Terminal cost only calculated for final state, not for intermediate switch states
    if (std::fabs(t - timeFinal_) > ocs2::numeric_traits::weakEpsilon<scalar_t>()) {
      return x;
    }

    vector_t xRef;
    const int currentMode = x.tail(1).value();
    ref_.getState(currentMode, t, xRef);
    return xRef;
  }

 private:
  OverallReference ref_;
  scalar_t timeFinal_;
  matrix_t QFinal_;
};
