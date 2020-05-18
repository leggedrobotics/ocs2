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

enum { STATE_DIM = 3, INPUT_DIM = 1 };

namespace ocs2 {

// #######################
// ###DYNAMICS CLASSES####
// #######################
class hybridSysDynamics1 final : public ControlledSystemBase {
 public:
  hybridSysDynamics1() = default;
  ~hybridSysDynamics1() override = default;

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) override {
    Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM> A;
    A << -0.1, 0.9, 0.0, -1, -0.01, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM> B;
    B << 0.0, 1.0, 0.0;
    Eigen::Matrix<scalar_t, STATE_DIM, 1> F;
    F << 0.0, 0.0, 0.0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const vector_t& state, vector_t& mappedState) override {
    mappedState.resize(STATE_DIM);
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 1;
  }

  void computeGuardSurfaces(const scalar_t& time, const vector_t& state, vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = 1;
    guardSurfacesValue[1] = -state[0] * state[1];
  }

  hybridSysDynamics1* clone() const override { return new hybridSysDynamics1(*this); }
};

class hybridSysDynamics2 final : public ControlledSystemBase {
 public:
  hybridSysDynamics2() = default;
  ~hybridSysDynamics2() override = default;

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) override {
    Eigen::Matrix<scalar_t, STATE_DIM, STATE_DIM> A;
    A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::Matrix<scalar_t, STATE_DIM, INPUT_DIM> B;
    B << 0.0, 1.0, 0.0;
    Eigen::Matrix<scalar_t, STATE_DIM, 1> F;
    F << 0.0, 0.0, 0.0;

    dxdt = A * x + B * u + F;
  }

  void computeJumpMap(const scalar_t& time, const vector_t& state, vector_t& mappedState) override {
    mappedState.resize(STATE_DIM);
    mappedState[0] = state[0];
    mappedState[1] = state[1];
    mappedState[2] = 0;
  }

  void computeGuardSurfaces(const scalar_t& time, const vector_t& state, vector_t& guardSurfacesValue) override {
    guardSurfacesValue.resize(2);
    guardSurfacesValue[0] = state[0] * state[1];
    guardSurfacesValue[1] = 1;
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

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) override {
    size_t activeSubsystem = x[2];
    subsystemDynamicsPtr_[activeSubsystem]->computeFlowMap(t, x, u, dxdt);
  }

  void computeJumpMap(const scalar_t& time, const vector_t& state, vector_t& mappedState) override {
    size_t activeSubsystem = state[2];
    subsystemDynamicsPtr_[activeSubsystem]->computeJumpMap(time, state, mappedState);
  }

  void computeGuardSurfaces(const scalar_t& time, const vector_t& state, vector_t& guardSurfacesValue) override {
    size_t activeSubsystem = state[2];
    subsystemDynamicsPtr_[activeSubsystem]->computeGuardSurfaces(time, state, guardSurfacesValue);
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

  void getFlowMapDerivativeState(matrix_t& A) override {
    A.resize(STATE_DIM, STATE_DIM);
    A << -0.1, 0.9, 0.0, -1.0, -0.01, 0.0, 0.0, 0.0, 0.0;
  }

  void getFlowMapDerivativeInput(matrix_t& B) override {
    B.resize(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
  }

  hybridSysDerivatives1* clone() const override { return new hybridSysDerivatives1(*this); }
};

class hybridSysDerivatives2 final : public DerivativesBase {
 public:
  hybridSysDerivatives2() = default;
  ~hybridSysDerivatives2() override = default;

  void getFlowMapDerivativeState(matrix_t& A) override {
    A.resize(STATE_DIM, STATE_DIM);
    A << -0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }

  void getFlowMapDerivativeInput(matrix_t& B) override {
    B.resize(STATE_DIM, INPUT_DIM);
    B << 0.0, 1.0, 0.0;
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

  void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) override {
    DerivativesBase::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = x[2];
    subsystemDerPtr_[activeSubsystem_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(matrix_t& A) override { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(matrix_t& B) override { subsystemDerPtr_[activeSubsystem_]->getFlowMapDerivativeInput(B); }

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

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero(STATE_DIM);
    dhdx[1].setZero(STATE_DIM);
    dhdx[2].resize(STATE_DIM);
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3].resize(STATE_DIM);
    dhdx[3] << -1.0, 0.0, 0.0;
  }

  void getInequalityConstraintDerivativesInput(vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0].resize(INPUT_DIM);
    dhdu[0] << -1.0;
    dhdu[1].resize(INPUT_DIM);
    dhdu[1] << 1.0;
    dhdu[2].setZero(INPUT_DIM);
    dhdu[3].setZero(INPUT_DIM);
  }

  void getInequalityConstraintSecondDerivativesState(matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[1].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[2].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[3].setZero(STATE_DIM, STATE_DIM);
  }

  void getInequalityConstraintSecondDerivativesInput(matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[1].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[2].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[3].setZero(INPUT_DIM, INPUT_DIM);
  }
  void getInequalityConstraintDerivativesInputState(matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[1].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[2].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[3].setZero(INPUT_DIM, STATE_DIM);
  }
};

class hybridSysConstraints2 final : public ConstraintBase {
 public:
  hybridSysConstraints2() : ConstraintBase(STATE_DIM, INPUT_DIM){};
  ~hybridSysConstraints2() override = default;

  hybridSysConstraints2* clone() const override { return new hybridSysConstraints2(*this); }

  void getInequalityConstraint(scalar_array_t& h) override {
    h.resize(4);
    h[0] = -u_[0] + 2;
    h[1] = u_[0] + 2;
    h[2] = x_[0] + 2;
    h[3] = -x_[0] + 2;
  }

  size_t numInequalityConstraint(const scalar_t& time) override { return 4; }

  void getInequalityConstraintDerivativesState(vector_array_t& dhdx) override {
    dhdx.resize(4);
    dhdx[0].setZero(STATE_DIM);
    dhdx[1].setZero(STATE_DIM);
    dhdx[2].resize(STATE_DIM);
    dhdx[2] << 1.0, 0.0, 0.0;
    dhdx[3].resize(STATE_DIM);
    dhdx[3] << -1.0, 0.0, 0.0;
  }

  void getInequalityConstraintDerivativesInput(vector_array_t& dhdu) override {
    dhdu.resize(4);
    dhdu[0].resize(INPUT_DIM);
    dhdu[0] << -1.0;
    dhdu[1].resize(INPUT_DIM);
    dhdu[1] << 1.0;
    dhdu[2].setZero(INPUT_DIM);
    dhdu[3].setZero(INPUT_DIM);
  }

  void getInequalityConstraintSecondDerivativesState(matrix_array_t& ddhdxdx) override {
    ddhdxdx.resize(4);
    ddhdxdx[0].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[1].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[2].setZero(STATE_DIM, STATE_DIM);
    ddhdxdx[3].setZero(STATE_DIM, STATE_DIM);
  }

  void getInequalityConstraintSecondDerivativesInput(matrix_array_t& ddhdudu) override {
    ddhdudu.resize(4);
    ddhdudu[0].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[1].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[2].setZero(INPUT_DIM, INPUT_DIM);
    ddhdudu[3].setZero(INPUT_DIM, INPUT_DIM);
  }
  void getInequalityConstraintDerivativesInputState(matrix_array_t& ddhdudx) override {
    ddhdudx.resize(4);
    ddhdudx[0].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[1].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[2].setZero(INPUT_DIM, STATE_DIM);
    ddhdudx[3].setZero(INPUT_DIM, STATE_DIM);
  }
};

class hybridSysConstraints final : public ConstraintBase {
 public:
  hybridSysConstraints() : ConstraintBase(3, 1), activeSubsystem_(1), subsystemConstPtr_(2) {
    subsystemConstPtr_[0].reset(new hybridSysConstraints1);
    subsystemConstPtr_[1].reset(new hybridSysConstraints2);
  }

  ~hybridSysConstraints() override = default;

  virtual void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) override {
    ConstraintBase::setCurrentStateAndControl(t, x, u);
    activeSubsystem_ = x[2];
    subsystemConstPtr_[activeSubsystem_]->setCurrentStateAndControl(t_, x_, u_);
  }

  void getInequalityConstraint(scalar_array_t& h) override { subsystemConstPtr_[activeSubsystem_]->getInequalityConstraint(h); }

  size_t numInequalityConstraint(const scalar_t& time) override {
    return subsystemConstPtr_[activeSubsystem_]->numInequalityConstraint(time);
  }

  void getInequalityConstraintDerivativesState(vector_array_t& dhdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesState(dhdx);
  }

  void getInequalityConstraintDerivativesInput(vector_array_t& dhdu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInput(dhdu);
  }

  void getInequalityConstraintSecondDerivativesInput(matrix_array_t& ddhdudu) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesInput(ddhdudu);
  }

  void getInequalityConstraintSecondDerivativesState(matrix_array_t& ddhdxdx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintSecondDerivativesState(ddhdxdx);
  }

  void getInequalityConstraintDerivativesInputState(matrix_array_t& ddhdudx) override {
    subsystemConstPtr_[activeSubsystem_]->getInequalityConstraintDerivativesInputState(ddhdudx);
  }

  hybridSysConstraints* clone() const override { return new hybridSysConstraints(*this); }

 private:
  int activeSubsystem_;
  std::vector<std::shared_ptr<ConstraintBase>> subsystemConstPtr_;
};

}  // namespace ocs2
