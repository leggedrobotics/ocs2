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
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

constexpr size_t STATE_DIM = 3;
constexpr size_t INPUT_DIM = 1;

namespace ocs2 {

/* Hybrid system dynamics */

class HybridSysDynamics1 final : public LinearSystemDynamics {
 public:
  HybridSysDynamics1() : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM)) {
    LinearSystemDynamics::A_ << -0.1, 0.9, 0.0, -1.0, -0.01, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;
  };
  ~HybridSysDynamics1() override = default;

  vector_t computeJumpMap(scalar_t time, const vector_t& state, const PreComputation&) override {
    vector_t mappedState = state;
    mappedState[2] = 1;
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t guardSurfaces(2);
    guardSurfaces[0] = 1;
    guardSurfaces[1] = -state[0] * state[1];
    return guardSurfaces;
  }

  HybridSysDynamics1* clone() const override { return new HybridSysDynamics1(*this); }
};

class HybridSysDynamics2 final : public LinearSystemDynamics {
 public:
  HybridSysDynamics2() : LinearSystemDynamics(matrix_t(STATE_DIM, STATE_DIM), matrix_t(STATE_DIM, INPUT_DIM)) {
    LinearSystemDynamics::A_ << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    LinearSystemDynamics::B_ << 0.0, 1.0, 0.0;
  };
  ~HybridSysDynamics2() override = default;

  vector_t computeJumpMap(scalar_t time, const vector_t& state, const PreComputation&) override {
    vector_t mappedState = state;
    mappedState[2] = 0;
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t guardSurfaces(2);
    guardSurfaces[0] = state[0] * state[1];
    guardSurfaces[1] = 1;
    return guardSurfaces;
  }

  HybridSysDynamics2* clone() const final { return new HybridSysDynamics2(*this); }
};

class HybridSysDynamics final : public SystemDynamicsBase {
 public:
  HybridSysDynamics() {
    subsystemDynamicsPtr_[0].reset(new HybridSysDynamics1);
    subsystemDynamicsPtr_[1].reset(new HybridSysDynamics2);
  }

  ~HybridSysDynamics() override = default;

  HybridSysDynamics* clone() const override { return new HybridSysDynamics(*this); }

  HybridSysDynamics(const HybridSysDynamics& other) : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
    subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
  }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) override {
    size_t activeSubsystem = x[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeFlowMap(t, x, u, preComp);
  }

  vector_t computeJumpMap(scalar_t time, const vector_t& state, const PreComputation& preComp) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeJumpMap(time, state, preComp);
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeGuardSurfaces(time, state);
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
    size_t activeSubsystem = x[2];
    return subsystemDynamicsPtr_[activeSubsystem]->linearApproximation(t, x, u, preComp);
  }

 private:
  std::vector<std::unique_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_{2};
};

/* Hybrid system constraints */

class HybridSysBounds final : public StateInputConstraint {
 public:
  HybridSysBounds() : StateInputConstraint(ConstraintOrder::Quadratic) {}
  ~HybridSysBounds() override = default;

  HybridSysBounds* clone() const override { return new HybridSysBounds(*this); }

  size_t getNumConstraints(scalar_t time) const override { return 4; }

  vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override {
    vector_t h(4);
    // clang-format off
    h << -u[0] + 2, // -2 < u < 2
          u[0] + 2,
          x[0] + 2, // -2 < x < 2
         -x[0] + 2;
    // clang-format on
    return h;
  }

  VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                 const PreComputation& preComp) const override {
    VectorFunctionQuadraticApproximation h;
    h.f = getValue(t, x, u, preComp);
    h.dfdx.setZero(4, STATE_DIM);
    h.dfdx.row(2) << 1, 0, 0;
    h.dfdx.row(3) << -1, 0, 0;
    h.dfdu.setZero(4, INPUT_DIM);
    h.dfdu << -1, 1, 0, 0;
    h.dfdxx.resize(4, matrix_t::Zero(STATE_DIM, STATE_DIM));
    h.dfduu.resize(4, matrix_t::Zero(INPUT_DIM, INPUT_DIM));
    h.dfdux.resize(4, matrix_t::Zero(INPUT_DIM, STATE_DIM));
    return h;
  }
};

}  // namespace ocs2
