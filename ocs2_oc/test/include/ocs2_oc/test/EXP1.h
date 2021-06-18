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

#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys1 final : public SystemDynamicsBase {
 public:
  EXP1_Sys1() = default;
  ~EXP1_Sys1() override = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    vector_t dxdt(2);
    dxdt(0) = x(0) + u(0) * sin(x(0));
    dxdt(1) = -x(1) - u(0) * cos(x(1));
    return dxdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << u(0) * cos(x(0)) + 1, 0, 0, u(0) * sin(x(1)) - 1;
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << sin(x(0)), -cos(x(1));
    return dynamics;
  }

  EXP1_Sys1* clone() const final { return new EXP1_Sys1(*this); }

 private:
  EXP1_Sys1(const EXP1_Sys1& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys2 final : public SystemDynamicsBase {
 public:
  EXP1_Sys2() = default;
  ~EXP1_Sys2() = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    vector_t dxdt(2);
    dxdt(0) = x(1) + u(0) * sin(x(1));
    dxdt(1) = -x(0) - u(0) * cos(x(0));
    return dxdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << 0, u(0) * cos(x(1)) + 1, u(0) * sin(x(0)) - 1, 0;
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << sin(x(1)), -cos(x(0));
    return dynamics;
  }

  EXP1_Sys2* clone() const final { return new EXP1_Sys2(*this); }

 private:
  EXP1_Sys2(const EXP1_Sys2& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_Sys3 final : public SystemDynamicsBase {
 public:
  EXP1_Sys3() = default;
  ~EXP1_Sys3() = default;

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    vector_t dxdt(2);
    dxdt(0) = -x(0) - u(0) * sin(x(0));
    dxdt(1) = x(1) + u(0) * cos(x(1));
    return dxdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u);
    dynamics.dfdx.resize(2, 2);
    dynamics.dfdx << -u(0) * cos(x(0)) - 1, 0, 0, 1 - u(0) * sin(x(1));
    dynamics.dfdu.resize(2, 1);
    dynamics.dfdu << -sin(x(0)), cos(x(1));
    return dynamics;
  }

  EXP1_Sys3* clone() const final { return new EXP1_Sys3(*this); }

 private:
  EXP1_Sys3(const EXP1_Sys3& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_System final : public SystemDynamicsBase {
 public:
  EXP1_System(std::shared_ptr<ReferenceManager> referenceManagerPtr) : referenceManagerPtr_(std::move(referenceManagerPtr)) {
    subsystemDynamicsPtr_[0].reset(new EXP1_Sys1());
    subsystemDynamicsPtr_[1].reset(new EXP1_Sys2());
    subsystemDynamicsPtr_[2].reset(new EXP1_Sys3());
  }

  ~EXP1_System() = default;

  EXP1_System* clone() const final { return new EXP1_System(*this); }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u);
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->linearApproximation(t, x, u);
  }

 private:
  EXP1_System(const EXP1_System& other) : EXP1_System(other.referenceManagerPtr_) {}

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_{3};
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP1_CostFunction : public CostFunctionBase {
 public:
  explicit EXP1_CostFunction(std::shared_ptr<ReferenceManager> referenceManagerPtr) : referenceManagerPtr_(std::move(referenceManagerPtr)) {
    const matrix_t Q = (matrix_t(2, 2) << 1.0, 0.0, 0.0, 1.0).finished();
    const matrix_t R = (matrix_t(1, 1) << 1.0).finished();
    const matrix_t Qf = (matrix_t(2, 2) << 1.0, 0.0, 0.0, 1.0).finished();
    subsystemCostsPtr_[0].reset(new QuadraticCostFunction(Q, R, matrix_t::Zero(2, 2)));
    subsystemCostsPtr_[1].reset(new QuadraticCostFunction(Q, R, matrix_t::Zero(2, 2)));
    subsystemCostsPtr_[2].reset(new QuadraticCostFunction(Q, R, Qf));

    subsystemCostsPtr_[0]->setTargetTrajectoriesPtr(&referenceManagerPtr_->getTargetTrajectories());
    subsystemCostsPtr_[1]->setTargetTrajectoriesPtr(&referenceManagerPtr_->getTargetTrajectories());
    subsystemCostsPtr_[2]->setTargetTrajectoriesPtr(&referenceManagerPtr_->getTargetTrajectories());
  }

  ~EXP1_CostFunction() = default;

  EXP1_CostFunction* clone() const final { return new EXP1_CostFunction(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->cost(t, x, u);
  }

  scalar_t finalCost(scalar_t t, const vector_t& x) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->finalCost(t, x);
  }

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->costQuadraticApproximation(t, x, u);
  }

  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->finalCostQuadraticApproximation(t, x);
  }

 private:
  EXP1_CostFunction(const EXP1_CostFunction& other) : EXP1_CostFunction(other.referenceManagerPtr_) {}

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::shared_ptr<CostFunctionBase>> subsystemCostsPtr_{3};
  TargetTrajectories targetTrajectories_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::shared_ptr<ReferenceManager> getExp1ReferenceManager(scalar_array_t eventTimes, std::vector<size_t> modeSequence) {
  const vector_t x = (vector_t(2) << 1.0, -1.0).finished();
  const vector_t u = (vector_t(1) << 0.0).finished();
  TargetTrajectories targetTrajectories({0.0}, {x}, {u});

  ModeSchedule modeSchedule(std::move(eventTimes), std::move(modeSequence));

  return std::make_shared<ReferenceManager>(std::move(targetTrajectories), std::move(modeSchedule));
}

}  // namespace ocs2
