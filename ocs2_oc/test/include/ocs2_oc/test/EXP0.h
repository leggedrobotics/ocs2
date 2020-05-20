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
#include <limits>
#include <memory>

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>

#include <ocs2_oc/oc_solver/ModeScheduleManager.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys1 : public ControlledSystemBase {
 public:
  EXP0_Sys1() = default;
  ~EXP0_Sys1() = default;

  void computeFlowMap(const double& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    matrix_t A(2, 2);
    A << 0.6, 1.2, -0.8, 3.4;
    vector_t B(2, 1);
    B << 1, 1;

    dxdt = A * x + B * u;
  }

  EXP0_Sys1* clone() const final { return new EXP0_Sys1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys2 : public ControlledSystemBase {
 public:
  EXP0_Sys2() = default;
  ~EXP0_Sys2() = default;

  void computeFlowMap(const double& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    matrix_t A(2, 2);
    A << 4, 3, -1, 0;
    vector_t B(2, 1);
    B << 2, -1;

    dxdt = A * x + B * u;
  }

  EXP0_Sys2* clone() const final { return new EXP0_Sys2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_System : public ControlledSystemBase {
 public:
  explicit EXP0_System(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(new EXP0_Sys1);
    subsystemDynamicsPtr_[1].reset(new EXP0_Sys2);
  }

  ~EXP0_System() override = default;

  EXP0_System(const EXP0_System& other) : EXP0_System(other.modeScheduleManagerPtr_) {}

  EXP0_System* clone() const final { return new EXP0_System(*this); }

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u, dxdt);
  }

 public:
  std::vector<std::shared_ptr<ControlledSystemBase>> subsystemDynamicsPtr_;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SysDerivative1 : public DerivativesBase {
 public:
  EXP0_SysDerivative1() = default;
  ~EXP0_SysDerivative1() = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0.6, 1.2, -0.8, 3.4;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 1, 1;
  }

  EXP0_SysDerivative1* clone() const final { return new EXP0_SysDerivative1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SysDerivative2 : public DerivativesBase {
 public:
  EXP0_SysDerivative2() = default;
  ~EXP0_SysDerivative2() = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 4, 3, -1, 0;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << 2, -1;
  }

  EXP0_SysDerivative2* clone() const final { return new EXP0_SysDerivative2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_SystemDerivative : public DerivativesBase {
 public:
  explicit EXP0_SystemDerivative(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDerivativesPtr_(2) {
    subsystemDerivativesPtr_[0].reset(new EXP0_SysDerivative1);
    subsystemDerivativesPtr_[1].reset(new EXP0_SysDerivative2);
  }

  ~EXP0_SystemDerivative() override = default;

  EXP0_SystemDerivative(const EXP0_SystemDerivative& other) : EXP0_SystemDerivative(other.modeScheduleManagerPtr_) {}

  EXP0_SystemDerivative* clone() const final { return new EXP0_SystemDerivative(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) final {
    DerivativesBase::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDerivativesPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(matrix_t& A) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(matrix_t& B) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeInput(B); }

 public:
  size_t activeMode_ = 0;
  std::vector<std::shared_ptr<DerivativesBase>> subsystemDerivativesPtr_;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP0_SystemConstraint = ConstraintBase;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction1 : public CostFunctionBase {
 public:
  EXP0_CostFunction1() = default;
  ~EXP0_CostFunction1() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0) + 0.5 * u_(0) * u_(0); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    dLdx.resize(2);
    dLdx << 0.0, (x_(1) - 2.0);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    dLdxx.resize(2, 2);
    dLdxx << 0.0, 0.0, 0.0, 1.0;
  }
  void getIntermediateCostDerivativeInput(vector_t& dLdu) final {
    dLdu.resize(1);
    dLdu << u_;
  }
  void getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) final {
    dLduu.resize(1, 1);
    dLduu << 1.0;
  }

  void getIntermediateCostDerivativeInputState(matrix_t& dLdxu) final { dLdxu.setZero(1, 2); }

  void getTerminalCost(scalar_t& Phi) final { Phi = 0; }
  void getTerminalCostDerivativeState(vector_t& dPhidx) final { dPhidx.setZero(2); }
  void getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) final { dPhidxx.setZero(2, 2); }

  EXP0_CostFunction1* clone() const final { return new EXP0_CostFunction1(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction2 : public CostFunctionBase {
 public:
  EXP0_CostFunction2() = default;
  ~EXP0_CostFunction2() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0) + 0.5 * u_(0) * u_(0); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    dLdx.resize(2);
    dLdx << 0.0, (x_(1) - 2.0);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    dLdxx.resize(2, 2);
    dLdxx << 0.0, 0.0, 0.0, 1.0;
  }
  void getIntermediateCostDerivativeInput(vector_t& dLdu) final {
    dLdu.resize(1);
    dLdu << u_;
  }
  void getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) final {
    dLduu.resize(1, 1);
    dLduu << 1.0;
  }

  void getIntermediateCostDerivativeInputState(matrix_t& dLdxu) final { dLdxu.setZero(1, 2); }

  void getTerminalCost(scalar_t& Phi) final { Phi = 0.5 * (x_(0) - 4.0) * (x_(0) - 4.0) + 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0); }
  void getTerminalCostDerivativeState(vector_t& dPhidx) final {
    dPhidx.resize(2);
    dPhidx << (x_(0) - 4.0), (x_(1) - 2.0);
  }
  void getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) final { dPhidxx.setIdentity(2, 2); }

  EXP0_CostFunction2* clone() const final { return new EXP0_CostFunction2(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction : public CostFunctionBase {
 public:
  explicit EXP0_CostFunction(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemCostsPtr_(2) {
    subsystemCostsPtr_[0].reset(new EXP0_CostFunction1);
    subsystemCostsPtr_[1].reset(new EXP0_CostFunction2);
  }

  ~EXP0_CostFunction() {}

  EXP0_CostFunction(const EXP0_CostFunction& other) : EXP0_CostFunction(other.modeScheduleManagerPtr_) {}

  EXP0_CostFunction* clone() const final { return new EXP0_CostFunction(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) final {
    CostFunctionBase::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemCostsPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  void getIntermediateCost(scalar_t& L) final { subsystemCostsPtr_[activeMode_]->getIntermediateCost(L); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeState(dLdx);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostSecondDerivativeState(dLdxx);
  }
  void getIntermediateCostDerivativeInput(vector_t& dLdu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeInput(dLdu);
  }
  void getIntermediateCostSecondDerivativeInput(matrix_t& dLduu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostSecondDerivativeInput(dLduu);
  }

  void getIntermediateCostDerivativeInputState(matrix_t& dLdxu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeInputState(dLdxu);
  }

  void getTerminalCost(scalar_t& Phi) { subsystemCostsPtr_[activeMode_]->getTerminalCost(Phi); }
  void getTerminalCostDerivativeState(vector_t& dPhidx) final { subsystemCostsPtr_[activeMode_]->getTerminalCostDerivativeState(dPhidx); }
  void getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) final {
    subsystemCostsPtr_[activeMode_]->getTerminalCostSecondDerivativeState(dPhidxx);
  }

 public:
  size_t activeMode_ = 0;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<CostFunctionBase>> subsystemCostsPtr_;
};

}  // namespace ocs2
