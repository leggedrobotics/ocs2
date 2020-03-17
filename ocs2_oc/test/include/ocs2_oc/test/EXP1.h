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
#include <ocs2_core/initialization/SystemOperatingPoint.h>

#include <ocs2_oc/oc_solver/ModeScheduleManager.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys1 : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_Sys1() = default;
  ~EXP1_Sys1() = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) final {
    dxdt(0) = x(0) + u(0) * sin(x(0));
    dxdt(1) = -x(1) - u(0) * cos(x(1));
  }

  EXP1_Sys1* clone() const final { return new EXP1_Sys1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys2 : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_Sys2() = default;
  ~EXP1_Sys2() = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) final {
    dxdt(0) = x(1) + u(0) * sin(x(1));
    dxdt(1) = -x(0) - u(0) * cos(x(0));
  }

  EXP1_Sys2* clone() const final { return new EXP1_Sys2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys3 : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_Sys3() = default;
  ~EXP1_Sys3() = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) final {
    dxdt(0) = -x(0) - u(0) * sin(x(0));
    dxdt(1) = x(1) + u(0) * cos(x(1));
  }

  EXP1_Sys3* clone() const final { return new EXP1_Sys3(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_System : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = ControlledSystemBase<2, 1>;

  EXP1_System(std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDynamicsPtr_(3) {
    subsystemDynamicsPtr_[0].reset(new EXP1_Sys1);
    subsystemDynamicsPtr_[1].reset(new EXP1_Sys2);
    subsystemDynamicsPtr_[2].reset(new EXP1_Sys3);
  }

  ~EXP1_System() = default;

  EXP1_System(const EXP1_System& other) : EXP1_System(other.modeScheduleManagerPtr_) {}

  EXP1_System* clone() const final { return new EXP1_System(*this); }

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, const input_vector_t& u, state_vector_t& dxdt) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u, dxdt);
  }

 private:
  std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr_;
  std::vector<Base::Ptr> subsystemDynamicsPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative1 : public DerivativesBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_SysDerivative1() = default;
  ~EXP1_SysDerivative1() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) final { A << u_(0) * cos(x_(0)) + 1, 0, 0, u_(0) * sin(x_(1)) - 1; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << sin(x_(0)), -cos(x_(1)); }

  EXP1_SysDerivative1* clone() const final { return new EXP1_SysDerivative1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative2 : public DerivativesBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_SysDerivative2() = default;
  ~EXP1_SysDerivative2() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) final { A << 0, u_(0) * cos(x_(1)) + 1, u_(0) * sin(x_(0)) - 1, 0; }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << sin(x_(1)), -cos(x_(0)); }

  EXP1_SysDerivative2* clone() const final { return new EXP1_SysDerivative2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SysDerivative3 : public DerivativesBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_SysDerivative3() = default;
  ~EXP1_SysDerivative3() = default;

  void getFlowMapDerivativeState(state_matrix_t& A) final { A << -u_(0) * cos(x_(0)) - 1, 0, 0, 1 - u_(0) * sin(x_(1)); }
  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { B << -sin(x_(0)), cos(x_(1)); }

  EXP1_SysDerivative3* clone() const final { return new EXP1_SysDerivative3(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_SystemDerivative : public DerivativesBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = DerivativesBase<2, 1>;

  EXP1_SystemDerivative(std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemDerivativesPtr_(3) {
    subsystemDerivativesPtr_[0].reset(new EXP1_SysDerivative1);
    subsystemDerivativesPtr_[1].reset(new EXP1_SysDerivative2);
    subsystemDerivativesPtr_[2].reset(new EXP1_SysDerivative3);
  }

  ~EXP1_SystemDerivative() = default;

  EXP1_SystemDerivative(const EXP1_SystemDerivative& other) : EXP1_SystemDerivative(other.modeScheduleManagerPtr_) {}

  EXP1_SystemDerivative* clone() const final { return new EXP1_SystemDerivative(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemDerivativesPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  void getFlowMapDerivativeState(state_matrix_t& A) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeState(A); }

  void getFlowMapDerivativeInput(state_input_matrix_t& B) final { subsystemDerivativesPtr_[activeMode_]->getFlowMapDerivativeInput(B); }

 private:
  size_t activeMode_ = 0;
  std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr_;
  std::vector<Base::Ptr> subsystemDerivativesPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemConstraint = ConstraintBase<2, 1>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction1 : public CostFunctionBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_CostFunction1() = default;
  ~EXP1_CostFunction1() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << (x_(0) - 1.0), (x_(1) + 1.0); }
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 1.0, 0.0, 0.0, 1.0; }
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << u_; }
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 1.0; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

  void getTerminalCost(scalar_t& Phi) { Phi = 0; }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx.setZero(); }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx.setZero(); }

  EXP1_CostFunction1* clone() const final { return new EXP1_CostFunction1(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction2 : public CostFunctionBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_CostFunction2() = default;
  ~EXP1_CostFunction2() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << (x_(0) - 1.0), (x_(1) + 1.0); }
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 1.0, 0.0, 0.0, 1.0; }
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << u_; }
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 1.0; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

  void getTerminalCost(scalar_t& Phi) final { Phi = 0; }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx.setZero(); }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx.setZero(); }

  EXP1_CostFunction2* clone() const final { return new EXP1_CostFunction2(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction3 : public CostFunctionBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EXP1_CostFunction3() = default;
  ~EXP1_CostFunction3() = default;

  void getIntermediateCost(scalar_t& L) final { L = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2) + 0.5 * pow(u_(0), 2); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final { dLdx << (x_(0) - 1.0), (x_(1) + 1.0); }
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final { dLdxx << 1.0, 0.0, 0.0, 1.0; }
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final { dLdu << u_; }
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final { dLduu << 1.0; }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final { dLdxu.setZero(); }

  void getTerminalCost(scalar_t& Phi) final { Phi = 0.5 * pow(x_(0) - 1.0, 2) + 0.5 * pow(x_(1) + 1.0, 2); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final { dPhidx << (x_(0) - 1.0), (x_(1) + 1.0); }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final { dPhidxx << 1.0, 0.0, 0.0, 1.0; }

  EXP1_CostFunction3* clone() const final { return new EXP1_CostFunction3(*this); };
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction : public CostFunctionBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = CostFunctionBase<2, 1>;

  explicit EXP1_CostFunction(std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)), subsystemCostsPtr_(3) {
    subsystemCostsPtr_[0].reset(new EXP1_CostFunction1);
    subsystemCostsPtr_[1].reset(new EXP1_CostFunction2);
    subsystemCostsPtr_[2].reset(new EXP1_CostFunction3);
  }

  ~EXP1_CostFunction() = default;

  EXP1_CostFunction(const EXP1_CostFunction& other) : EXP1_CostFunction(other.modeScheduleManagerPtr_) {}

  EXP1_CostFunction* clone() const final { return new EXP1_CostFunction(*this); }

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) final {
    Base::setCurrentStateAndControl(t, x, u);
    activeMode_ = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    subsystemCostsPtr_[activeMode_]->setCurrentStateAndControl(t, x, u);
  }

  void getIntermediateCost(scalar_t& L) final { subsystemCostsPtr_[activeMode_]->getIntermediateCost(L); }

  void getIntermediateCostDerivativeState(state_vector_t& dLdx) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeState(dLdx);
  }
  void getIntermediateCostSecondDerivativeState(state_matrix_t& dLdxx) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostSecondDerivativeState(dLdxx);
  }
  void getIntermediateCostDerivativeInput(input_vector_t& dLdu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeInput(dLdu);
  }
  void getIntermediateCostSecondDerivativeInput(input_matrix_t& dLduu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostSecondDerivativeInput(dLduu);
  }

  void getIntermediateCostDerivativeInputState(input_state_matrix_t& dLdxu) final {
    subsystemCostsPtr_[activeMode_]->getIntermediateCostDerivativeInputState(dLdxu);
  }

  void getTerminalCost(scalar_t& Phi) final { subsystemCostsPtr_[activeMode_]->getTerminalCost(Phi); }
  void getTerminalCostDerivativeState(state_vector_t& dPhidx) final {
    subsystemCostsPtr_[activeMode_]->getTerminalCostDerivativeState(dPhidx);
  }
  void getTerminalCostSecondDerivativeState(state_matrix_t& dPhidxx) final {
    subsystemCostsPtr_[activeMode_]->getTerminalCostSecondDerivativeState(dPhidxx);
  }

 public:
  size_t activeMode_ = 0;
  std::shared_ptr<ModeScheduleManager<2, 1>> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<Base>> subsystemCostsPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
using EXP1_SystemOperatingTrajectories = SystemOperatingPoint<2, 1>;

}  // namespace ocs2
