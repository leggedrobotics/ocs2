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
class EXP1_Sys1 : public ControlledSystemBase {
 public:
  EXP1_Sys1() = default;
  ~EXP1_Sys1() override = default;

  void computeFlowMap(const double& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    dxdt.resize(2);
    dxdt(0) = x(0) + u(0) * sin(x(0));
    dxdt(1) = -x(1) - u(0) * cos(x(1));
  }

  EXP1_Sys1* clone() const final { return new EXP1_Sys1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys2 : public ControlledSystemBase {
 public:
  EXP1_Sys2() = default;
  ~EXP1_Sys2() = default;

  void computeFlowMap(const double& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    dxdt.resize(2);
    dxdt(0) = x(1) + u(0) * sin(x(1));
    dxdt(1) = -x(0) - u(0) * cos(x(0));
  }

  EXP1_Sys2* clone() const final { return new EXP1_Sys2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys3 : public ControlledSystemBase {
 public:
  EXP1_Sys3() = default;
  ~EXP1_Sys3() = default;

  void computeFlowMap(const double& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    dxdt.resize(2);
    dxdt(0) = -x(0) - u(0) * sin(x(0));
    dxdt(1) = x(1) + u(0) * cos(x(1));
  }

  EXP1_Sys3* clone() const final { return new EXP1_Sys3(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_System : public ControlledSystemBase {
 public:
  EXP1_System(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDynamicsPtr_(3) {
    subsystemDynamicsPtr_[0].reset(new EXP1_Sys1);
    subsystemDynamicsPtr_[1].reset(new EXP1_Sys2);
    subsystemDynamicsPtr_[2].reset(new EXP1_Sys3);
  }

  ~EXP1_System() = default;

  EXP1_System(const EXP1_System& other) : EXP1_System(other.modeScheduleManagerPtr_) {}

  EXP1_System* clone() const final { return new EXP1_System(*this); }

  void computeFlowMap(const scalar_t& t, const vector_t& x, const vector_t& u, vector_t& dxdt) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u, dxdt);
  }

 private:
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<ControlledSystemBase>> subsystemDynamicsPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative1 : public DerivativesBase {
 public:
  EXP1_SysDerivative1() = default;
  ~EXP1_SysDerivative1() = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << u_(0) * cos(x_(0)) + 1, 0, 0, u_(0) * sin(x_(1)) - 1;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << sin(x_(0)), -cos(x_(1));
  }

  EXP1_SysDerivative1* clone() const final { return new EXP1_SysDerivative1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative2 : public DerivativesBase {
 public:
  EXP1_SysDerivative2() = default;
  ~EXP1_SysDerivative2() = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << 0, u_(0) * cos(x_(1)) + 1, u_(0) * sin(x_(0)) - 1, 0;
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << sin(x_(1)), -cos(x_(0));
  }

  EXP1_SysDerivative2* clone() const final { return new EXP1_SysDerivative2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative3 : public DerivativesBase {
 public:
  EXP1_SysDerivative3() = default;
  ~EXP1_SysDerivative3() = default;

  void getFlowMapDerivativeState(matrix_t& A) final {
    A.resize(2, 2);
    A << -u_(0) * cos(x_(0)) - 1, 0, 0, 1 - u_(0) * sin(x_(1));
  }
  void getFlowMapDerivativeInput(matrix_t& B) final {
    B.resize(2, 1);
    B << -sin(x_(0)), cos(x_(1));
  }

  EXP1_SysDerivative3* clone() const final { return new EXP1_SysDerivative3(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SystemDerivative : public DerivativesBase {
 public:
  EXP1_SystemDerivative(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDerivativesPtr_(3) {
    subsystemDerivativesPtr_[0].reset(new EXP1_SysDerivative1);
    subsystemDerivativesPtr_[1].reset(new EXP1_SysDerivative2);
    subsystemDerivativesPtr_[2].reset(new EXP1_SysDerivative3);
  }

  ~EXP1_SystemDerivative() = default;

  EXP1_SystemDerivative(const EXP1_SystemDerivative& other) : EXP1_SystemDerivative(other.modeScheduleManagerPtr_) {}

  EXP1_SystemDerivative* clone() const final { return new EXP1_SystemDerivative(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const vector_t& x, const vector_t& u) final {
    DerivativesBase::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDerivativesPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(matrix_t& A) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(matrix_t& B) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeInput(B); }

 private:
  size_t activeMode_ = 0;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<DerivativesBase>> subsystemDerivativesPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemConstraint = ConstraintBase;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction1 : public CostFunctionBase {
 public:
  EXP1_CostFunction1() = default;
  ~EXP1_CostFunction1() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    dLdx.resize(2);
    dLdx << (x_(0) - 1.0), (x_(1) + 1.0);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    dLdxx.resize(2, 2);
    dLdxx << 1.0, 0.0, 0.0, 1.0;
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

  void getTerminalCost(scalar_t& Phi) { Phi = 0; }
  void getTerminalCostDerivativeState(vector_t& dPhidx) final { dPhidx.setZero(2); }
  void getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) final { dPhidxx.setZero(2, 2); }

  EXP1_CostFunction1* clone() const final { return new EXP1_CostFunction1(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction2 : public CostFunctionBase {
 public:
  EXP1_CostFunction2() = default;
  ~EXP1_CostFunction2() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    dLdx.resize(2);
    dLdx << (x_(0) - 1.0), (x_(1) + 1.0);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    dLdxx.resize(2, 2);
    dLdxx << 1.0, 0.0, 0.0, 1.0;
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

  EXP1_CostFunction2* clone() const final { return new EXP1_CostFunction2(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction3 : public CostFunctionBase {
 public:
  EXP1_CostFunction3() = default;
  ~EXP1_CostFunction3() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(vector_t& dLdx) final {
    dLdx.resize(2);
    dLdx << (x_(0) - 1.0), (x_(1) + 1.0);
  }
  void getIntermediateCostSecondDerivativeState(matrix_t& dLdxx) final {
    dLdxx.resize(2, 2);
    dLdxx << 1.0, 0.0, 0.0, 1.0;
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

  void getTerminalCost(scalar_t& Phi) final { Phi = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2); }
  void getTerminalCostDerivativeState(vector_t& dPhidx) final {
    dPhidx.resize(2);
    dPhidx << (x_(0) - 1.0), (x_(1) + 1.0);
  }
  void getTerminalCostSecondDerivativeState(matrix_t& dPhidxx) final {
    dPhidxx.resize(2, 2);
    dPhidxx << 1.0, 0.0, 0.0, 1.0;
  }

  EXP1_CostFunction3* clone() const final { return new EXP1_CostFunction3(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction : public CostFunctionBase {
 public:
  explicit EXP1_CostFunction(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemCostsPtr_(3) {
    subsystemCostsPtr_[0].reset(new EXP1_CostFunction1);
    subsystemCostsPtr_[1].reset(new EXP1_CostFunction2);
    subsystemCostsPtr_[2].reset(new EXP1_CostFunction3);
  }

  ~EXP1_CostFunction() = default;

  EXP1_CostFunction(const EXP1_CostFunction& other) : EXP1_CostFunction(other.modeScheduleManagerPtr_) {}

  EXP1_CostFunction* clone() const final { return new EXP1_CostFunction(*this); }

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

  void getTerminalCost(scalar_t& Phi) final { subsystemCostsPtr_[activeMode_]->getTerminalCost(Phi); }
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
