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

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    matrix_t A(2, 2);
    A << 0.6, 1.2, -0.8, 3.4;
    vector_t B(2, 1);
    B << 1, 1;

    return A * x + B * u;
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

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    matrix_t A(2, 2);
    A << 4, 3, -1, 0;
    vector_t B(2, 1);
    B << 2, -1;

    return A * x + B * u;
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

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u);
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

  matrix_t getFlowMapDerivativeState() final {
    matrix_t A(2, 2);
    A << 0.6, 1.2, -0.8, 3.4;
    return A;
  }
  matrix_t getFlowMapDerivativeInput() final {
    matrix_t B(2, 1);
    B << 1, 1;
    return B;
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

  matrix_t getFlowMapDerivativeState() final {
    matrix_t A(2, 2);
    A << 4, 3, -1, 0;
    return A;
  }
  matrix_t getFlowMapDerivativeInput() final {
    matrix_t B(2, 1);
    B << 2, -1;
    return B;
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

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) final {
    DerivativesBase::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDerivativesPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  matrix_t getFlowMapDerivativeState() final { return subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeState(); }

  matrix_t getFlowMapDerivativeInput() final { return subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeInput(); }

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

  scalar_t getCost() final { return 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0) + 0.5 * u_(0) * u_(0); }

  vector_t getCostDerivativeState() final {
    vector_t dLdx(2);
    dLdx << 0.0, (x_(1) - 2.0);
    return dLdx;
  }
  matrix_t getCostSecondDerivativeState() final {
    matrix_t dLdxx(2, 2);
    dLdxx << 0.0, 0.0, 0.0, 1.0;
    return dLdxx;
  }
  vector_t getCostDerivativeInput() final {
    vector_t dLdu(1);
    dLdu << u_;
    return dLdu;
  }
  matrix_t getCostSecondDerivativeInput() final {
    matrix_t dLduu(1, 1);
    dLduu << 1.0;
    return dLduu;
  }
  matrix_t getCostDerivativeInputState() final { return matrix_t::Zero(1, 2); }
  scalar_t getTerminalCost() final { return 0; }
  vector_t getTerminalCostDerivativeState() final { return vector_t::Zero(2); }
  matrix_t getTerminalCostSecondDerivativeState() final { return matrix_t::Zero(2, 2); }

  EXP0_CostFunction1* clone() const final { return new EXP0_CostFunction1(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction2 : public CostFunctionBase {
 public:
  EXP0_CostFunction2() = default;
  ~EXP0_CostFunction2() = default;

  scalar_t getCost() final { return 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0) + 0.5 * u_(0) * u_(0); }

  vector_t getCostDerivativeState() final {
    vector_t dLdx(2);
    dLdx << 0.0, (x_(1) - 2.0);
    return dLdx;
  }
  matrix_t getCostSecondDerivativeState() final {
    matrix_t dLdxx(2, 2);
    dLdxx << 0.0, 0.0, 0.0, 1.0;
    return dLdxx;
  }
  vector_t getCostDerivativeInput() final {
    vector_t dLdu(1);
    dLdu << u_;
    return dLdu;
  }
  matrix_t getCostSecondDerivativeInput() final {
    matrix_t dLduu(1, 1);
    dLduu << 1.0;
    return dLduu;
  }

  matrix_t getCostDerivativeInputState() final {
    matrix_t dLdxu;
    dLdxu.setZero(1, 2);
    return dLdxu;
  }
  scalar_t getTerminalCost() final { return 0.5 * (x_(0) - 4.0) * (x_(0) - 4.0) + 0.5 * (x_(1) - 2.0) * (x_(1) - 2.0); }
  vector_t getTerminalCostDerivativeState() final {
    vector_t dPhidx(2);
    dPhidx << (x_(0) - 4.0), (x_(1) - 2.0);
    return dPhidx;
  }
  matrix_t getTerminalCostSecondDerivativeState() final { return matrix_t::Identity(2, 2); }

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

  void setCurrentStateAndControl(scalar_t t, const vector_t& x, const vector_t& u) final {
    CostFunctionBase::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemCostsPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  scalar_t getCost() final { return subsystemCostsPtr_[activeMode_]->getCost(); }
  vector_t getCostDerivativeState() final { return subsystemCostsPtr_[activeMode_]->getCostDerivativeState(); }
  matrix_t getCostSecondDerivativeState() final { return subsystemCostsPtr_[activeMode_]->getCostSecondDerivativeState(); }
  vector_t getCostDerivativeInput() final { return subsystemCostsPtr_[activeMode_]->getCostDerivativeInput(); }
  matrix_t getCostSecondDerivativeInput() final { return subsystemCostsPtr_[activeMode_]->getCostSecondDerivativeInput(); }
  matrix_t getCostDerivativeInputState() final { return subsystemCostsPtr_[activeMode_]->getCostDerivativeInputState(); }
  scalar_t getTerminalCost() { return subsystemCostsPtr_[activeMode_]->getTerminalCost(); }
  vector_t getTerminalCostDerivativeState() final { return subsystemCostsPtr_[activeMode_]->getTerminalCostDerivativeState(); }
  matrix_t getTerminalCostSecondDerivativeState() final { return subsystemCostsPtr_[activeMode_]->getTerminalCostSecondDerivativeState(); }

 public:
  size_t activeMode_ = 0;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<CostFunctionBase>> subsystemCostsPtr_;
};

}  // namespace ocs2
