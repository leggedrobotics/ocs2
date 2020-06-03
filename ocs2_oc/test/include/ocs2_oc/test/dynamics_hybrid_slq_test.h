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
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>

constexpr size_t STATE_DIM = 3;
constexpr size_t INPUT_DIM = 1;

namespace ocs2 {

// #######################
// ###DYNAMICS CLASSES####
// #######################
class hybridSysDynamics1 final : public ControlledSystemBase {
 public:
  hybridSysDynamics1() = default;
  ~hybridSysDynamics1() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) override {
    matrix_t A(STATE_DIM, STATE_DIM);
    A << -0.1, 0.9, 0.0, -1, -0.01, 0.0, 0.0, 0.0, 0.0;
    matrix_t B(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
    vector_t F(STATE_DIM);
    F << 0.0, 0.0, 0.0;

    return A * x + B * u + F;
  }

  vector_t computeJumpMap(scalar_t time, const vector_t& state) override {
    vector_t mappedState(STATE_DIM);
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 1;
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t guardSurfaces(2);
    guardSurfaces[0] = 1;
    guardSurfaces[1] = -state[0] * state[1];
    return guardSurfaces;
  }

  hybridSysDynamics1* clone() const override { return new hybridSysDynamics1(*this); }
};

class hybridSysDynamics2 final : public ControlledSystemBase {
 public:
  hybridSysDynamics2() = default;
  ~hybridSysDynamics2() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) override {
    matrix_t A(STATE_DIM, STATE_DIM);
    A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    matrix_t B(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
    vector_t F(STATE_DIM);
    F << 0.0, 0.0, 0.0;

    return A * x + B * u + F;
  }

  vector_t computeJumpMap(scalar_t time, const vector_t& state) override {
    vector_t mappedState(STATE_DIM);
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 0;
    return mappedState;
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    vector_t guardSurfaces(2);
    guardSurfaces[0] = state[0] * state[1];
    guardSurfaces[1] = 1;
    return guardSurfaces;
  }

  hybridSysDynamics2* clone() const final { return new hybridSysDynamics2(*this); }
};

class hybridSysDynamics final : public ControlledSystemBase {
 public:
  hybridSysDynamics() : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(new hybridSysDynamics1);
    subsystemDynamicsPtr_[1].reset(new hybridSysDynamics2);
  }

  ~hybridSysDynamics() override = default;

  hybridSysDynamics* clone() const override { return new hybridSysDynamics(*this); }

  hybridSysDynamics(const hybridSysDynamics& other) : subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(other.subsystemDynamicsPtr_[0]->clone());
    subsystemDynamicsPtr_[1].reset(other.subsystemDynamicsPtr_[1]->clone());
  }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) override {
    size_t activeSubsystem = x[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeFlowMap(t, x, u);
  }

  vector_t computeJumpMap(scalar_t time, const vector_t& state) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeJumpMap(time, state);
  }

  vector_t computeGuardSurfaces(scalar_t time, const vector_t& state) override {
    size_t activeSubsystem = state[2];
    return subsystemDynamicsPtr_[activeSubsystem]->computeGuardSurfaces(time, state);
  }

 private:
  std::vector<std::shared_ptr<ControlledSystemBase>> subsystemDynamicsPtr_;
};

// ############################
// ####DERIVATIVE CLASSES######
// ############################
class hybridSysDerivatives1 final : public DerivativesBase {
 public:
  hybridSysDerivatives1() = default;
  ~hybridSysDerivatives1() override = default;

  matrix_t getFlowMapDerivativeState() override {
    matrix_t A(STATE_DIM, STATE_DIM);
    A << -0.1, 0.9, 0.0, -1.0, -0.01, 0.0, 0.0, 0.0, 0.0;
    return A;
  }

  matrix_t getFlowMapDerivativeInput() override {
    matrix_t B(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
    return B;
  }

  hybridSysDerivatives1* clone() const override { return new hybridSysDerivatives1(*this); }
};

class hybridSysDerivatives2 final : public DerivativesBase {
 public:
  hybridSysDerivatives2() = default;
  ~hybridSysDerivatives2() override = default;

  matrix_t getFlowMapDerivativeState() override {
    matrix_t A(STATE_DIM, STATE_DIM);
    A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    return A;
  }

  matrix_t getFlowMapDerivativeInput() override {
    matrix_t B(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
    return B;
  }

  hybridSysDerivatives2* clone() const override { return new hybridSysDerivatives2(*this); }
};

class hybridSysDerivatives final : public DerivativesBase {
 public:
  hybridSysDerivatives() : activeSubsystem_(1), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(new hybridSysDerivatives1);
    subsystemDerPtr_[1].reset(new hybridSysDerivatives2);
  }

  ~hybridSysDerivatives() override = default;

  hybridSysDerivatives* clone() const final { return new hybridSysDerivatives(*this); }

  hybridSysDerivatives(const hybridSysDerivatives& other) : activeSubsystem_(other.activeSubsystem_), subsystemDerPtr_(2) {
    subsystemDerPtr_[0].reset(other.subsystemDerPtr_[0]->clone());
    subsystemDerPtr_[1].reset(other.subsystemDerPtr_[1]->clone());
  }

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override {
    DerivativesBase::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = x[2];
    subsystemDerPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }

  matrix_t getFlowMapDerivativeState() override { return subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeState(); }

  matrix_t getFlowMapDerivativeInput() override { return subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeInput(); }

 private:
  int activeSubsystem_;
  std::vector<std::shared_ptr<DerivativesBase>> subsystemDerPtr_;
};

// #############################
// #### CONSTRAINT CLASSES######
// #############################
class hybridSysConstraints1 final : public ConstraintBase {
 public:
  hybridSysConstraints1() : ConstraintBase(STATE_DIM, INPUT_DIM){};
  ~hybridSysConstraints1() = default;

  hybridSysConstraints1* clone() const override { return new hybridSysConstraints1(*this); }

  scalar_array_t getInequalityConstraint() override {
    scalar_array_t h;
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
    return h;
  }

  vector_array_t getInequalityConstraintDerivativesState() override {
    vector_array_t dhdx;
    dhdx.resize(4);
    dhdx[0].setZero(STATE_DIM);
    dhdx[1].setZero(STATE_DIM);
    dhdx[2].resize(STATE_DIM);
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3].resize(STATE_DIM);
    dhdx[3] << -1.0, 0.0, 0.0;
    return dhdx;
  }

  vector_array_t getInequalityConstraintDerivativesInput() override {
    vector_array_t dhdu;
    dhdu.resize(4);
    dhdu[0].resize(INPUT_DIM);
    dhdu[0] << -1.0;
    dhdu[1].resize(INPUT_DIM);
    dhdu[1] << 1.0;
    dhdu[2].setZero(INPUT_DIM);
    dhdu[3].setZero(INPUT_DIM);
    return dhdu;
  }

  matrix_array_t getInequalityConstraintSecondDerivativesState() override {
    matrix_array_t ddhdxdx;
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[1].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[2].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[3].setZero(STATE_DIM, STATE_DIM);
    return ddhdxdx;
  }

  matrix_array_t getInequalityConstraintSecondDerivativesInput() override {
    matrix_array_t ddhdudu;
    ddhdudu.resize(4);
    ddhdudu[0].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[1].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[2].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[3].setZero(INPUT_DIM, INPUT_DIM);
    return ddhdudu;
  }

  matrix_array_t getInequalityConstraintDerivativesInputState() override {
    matrix_array_t ddhdudx;
    ddhdudx.resize(4);
    ddhdudx[0].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[1].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[2].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[3].setZero(INPUT_DIM, STATE_DIM);
    return ddhdudx;
  }
};

class hybridSysConstraints2 final : public ConstraintBase {
 public:
  hybridSysConstraints2() : ConstraintBase(STATE_DIM, INPUT_DIM){};
  ~hybridSysConstraints2() override = default;

  hybridSysConstraints2* clone() const override { return new hybridSysConstraints2(*this); }

  scalar_array_t getInequalityConstraint() override {
    scalar_array_t h;
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
    return h;
  }

  vector_array_t getInequalityConstraintDerivativesState() override {
    vector_array_t dhdx;
    dhdx.resize(4);
    dhdx[0].setZero(STATE_DIM);
    dhdx[1].setZero(STATE_DIM);
    dhdx[2].resize(STATE_DIM);
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3].resize(STATE_DIM);
    dhdx[3] << -1.0, 0.0, 0.0;
    return dhdx;
  }

  vector_array_t getInequalityConstraintDerivativesInput() override {
    vector_array_t dhdu;
    dhdu.resize(4);
    dhdu[0].resize(INPUT_DIM);
    dhdu[0] << -1.0;
    dhdu[1].resize(INPUT_DIM);
    dhdu[1] << 1.0;
    dhdu[2].setZero(INPUT_DIM);
    dhdu[3].setZero(INPUT_DIM);
    return dhdu;
  }

  matrix_array_t getInequalityConstraintSecondDerivativesState() override {
    matrix_array_t ddhdxdx;
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[1].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[2].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[3].setZero(STATE_DIM, STATE_DIM);
    return ddhdxdx;
  }

  matrix_array_t getInequalityConstraintSecondDerivativesInput() override {
    matrix_array_t ddhdudu;
    ddhdudu.resize(4);
    ddhdudu[0].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[1].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[2].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[3].setZero(INPUT_DIM, INPUT_DIM);
    return ddhdudu;
  }

  matrix_array_t getInequalityConstraintDerivativesInputState() override {
    matrix_array_t ddhdudx;
    ddhdudx.resize(4);
    ddhdudx[0].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[1].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[2].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[3].setZero(INPUT_DIM, STATE_DIM);
    return ddhdudx;
  }
};

class hybridSysConstraints final : public ConstraintBase {
 public:
  hybridSysConstraints() : ConstraintBase(3, 1), activeSubsystem_(1), subsystemConstPtr_(2) {
    subsystemConstPtr_[0].reset(new hybridSysConstraints1);
    subsystemConstPtr_[1].reset(new hybridSysConstraints2);
  }

  ~hybridSysConstraints() override = default;

  virtual void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) override {
    ConstraintBase::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = x[2];
    subsystemConstPtr_[activeSubsystem_]->setCurrentStateAndControl(t_, x_, u_);
  }

  scalar_array_t getInequalityConstraint() override { return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraint(); }

  vector_array_t getInequalityConstraintDerivativesState() override {
    return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesState();
  }

  vector_array_t getInequalityConstraintDerivativesInput() override {
    return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInput();
  }

  matrix_array_t getInequalityConstraintSecondDerivativesInput() override {
    return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesInput();
  }

  matrix_array_t getInequalityConstraintSecondDerivativesState() override {
    return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesState();
  }

  matrix_array_t getInequalityConstraintDerivativesInputState() override {
    return subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInputState();
  }

  hybridSysConstraints* clone() const override { return new hybridSysConstraints(*this); }

 private:
  int activeSubsystem_;
  std::vector<std::shared_ptr<ConstraintBase>> subsystemConstPtr_;
};

}  // namespace ocs2
