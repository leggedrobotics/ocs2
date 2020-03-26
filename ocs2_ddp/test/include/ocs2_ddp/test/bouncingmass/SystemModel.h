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

#include <Eigen/Dense>
#include <iostream>
#include <vector>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/control/LinearController.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_ddp/SLQ_Settings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_ddp/test/bouncingmass/OverallReference.h"

enum { STATE_DIM = 3, INPUT_DIM = 1 };

using DIMENSIONS = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
using scalar_t = typename DIMENSIONS::scalar_t;
using state_vector_t = typename DIMENSIONS::state_vector_t;
using input_vector_t = typename DIMENSIONS::input_vector_t;
using dynamic_vec_t = typename DIMENSIONS::dynamic_vector_t;

using state_matrix_t = typename DIMENSIONS::state_matrix_t;
using input_matrix_t = typename DIMENSIONS::input_matrix_t;
using state_input_matrix_t = typename DIMENSIONS::state_input_matrix_t;

using scalar_array_t = typename DIMENSIONS::scalar_array_t;
using size_array_t = typename DIMENSIONS::size_array_t;

class systemDynamics final : public ocs2::ControlledSystemBase<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  systemDynamics() = default;
  ~systemDynamics() = default;

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) {
    state_matrix_t A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    state_input_matrix_t B;
    B << 0.0, 1.0, 0.0;

    dxdt = A * x + B * u;
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    const scalar_t e = 0.95;

    state_matrix_t delta;
    delta << 0.0, 0.0, 0.0, 0.0, -(1.0 + e), 0.0, 0.0, 0.0, 0.0;
    mappedState = state + delta * state;

    if (state[2] < 5) {
      mappedState[2] += 1;
    }
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) {
    guardSurfacesValue.resize(1);
    guardSurfacesValue[0] = state[0];
  }

  systemDynamics* clone() const final { return new systemDynamics(*this); }
};

class systemDerivative final : public ocs2::DerivativesBase<STATE_DIM, INPUT_DIM> {
 public:
  systemDerivative() = default;
  ~systemDerivative() = default;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& state, const input_vector_t& u) {
    t_ = t;
    state_ = state;
    u_ = u;
  }

  void getFlowMapDerivativeState(state_matrix_t& A) { A << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) { B << 0.0, 1.0, 0.0; }

  systemDerivative* clone() const final { return new systemDerivative(*this); }

 private:
  scalar_t t_;
  state_vector_t state_;
  input_vector_t u_;
};

class systemCost final : public ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM> {
 public:
  using BASE = ocs2::QuadraticCostFunction<STATE_DIM, INPUT_DIM>;

  systemCost(OverallReference ref, state_matrix_t Q, input_matrix_t R, state_matrix_t P, state_vector_t xNom, input_vector_t uNom,
             state_vector_t xFin, scalar_t timeFinal)
      : BASE(Q, R, xNom, uNom, P, xFin), ref_(ref), xFin_(xFin), timeFinal_(timeFinal) {}

  ~systemCost() = default;

  systemCost* clone() const final { return new systemCost(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& state, const input_vector_t& u) {
    t_ = t;
    state_ = state;
    u_ = u;

    int currentMode = state_.tail(1).value();
    ref_.getInput(t_, uRef_);
    ref_.getState(currentMode, t_, stateRef_);

    // Terminal cost only calculated for final state, not for intermediate switch states
    if (std::fabs(t - timeFinal_) > ocs2::OCS2NumericTraits<scalar_t>::weakEpsilon()) {
      BASE::setCurrentStateAndControl(t_, state_, u_, stateRef_, uRef_, state_);
    } else {
      BASE::setCurrentStateAndControl(t_, state_, u_, stateRef_, uRef_, stateRef_);
    }
  }

 private:
  scalar_t t_;
  state_vector_t state_;
  state_vector_t stateRef_;
  input_vector_t u_;
  input_vector_t uRef_;

  state_vector_t xFin_;
  scalar_t timeFinal_;

  OverallReference ref_;
};

using systemConstraint = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;

using systemOperatingTrajectories = ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
