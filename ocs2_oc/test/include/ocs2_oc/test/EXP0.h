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

#include <memory>

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys1 : public LinearSystemDynamics {
 public:
  EXP0_Sys1() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 0.6, 1.2, -0.8, 3.4;
    LinearSystemDynamics::B_ << 1, 1;
  };
  ~EXP0_Sys1() = default;

  EXP0_Sys1* clone() const final { return new EXP0_Sys1(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys2 : public LinearSystemDynamics {
 public:
  EXP0_Sys2() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 4, 3, -1, 0;
    LinearSystemDynamics::B_ << 2, -1;
  };
  ~EXP0_Sys2() = default;

  EXP0_Sys2* clone() const final { return new EXP0_Sys2(*this); }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_System : public SystemDynamicsBase {
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
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u);
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->linearApproximation(t, x, u);
  }

 public:
  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction : public CostFunctionBase {
 public:
  explicit EXP0_CostFunction(std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr)
      : modeScheduleManagerPtr_(std::move(modeScheduleManagerPtr)) {
    matrix_t Q(2, 2);
    matrix_t R(1, 1);
    matrix_t Qf(2, 2);
    Q << 0.0, 0.0, 0.0, 1.0;
    R << 1.0;
    Qf << 1.0, 0.0, 0.0, 1.0;
    subsystemCostsPtr_[0].reset(new QuadraticCostFunction(Q, R, matrix_t::Zero(2, 2)));
    subsystemCostsPtr_[1].reset(new QuadraticCostFunction(Q, R, Qf));

    vector_t x(2);
    vector_t u(1);
    x << 4.0, 2.0;
    u << 0.0;
    targetTrajectories_ = TargetTrajectories({0.0}, {x}, {u});
    subsystemCostsPtr_[0]->setTargetTrajectoriesPtr(&targetTrajectories_);
    subsystemCostsPtr_[1]->setTargetTrajectoriesPtr(&targetTrajectories_);
  }

  ~EXP0_CostFunction() = default;

  EXP0_CostFunction(const EXP0_CostFunction& other) : EXP0_CostFunction(other.modeScheduleManagerPtr_) {}

  EXP0_CostFunction* clone() const final { return new EXP0_CostFunction(*this); }

  scalar_t cost(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->cost(t, x, u);
  }

  scalar_t finalCost(scalar_t t, const vector_t& x) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->finalCost(t, x);
  }

  ScalarFunctionQuadraticApproximation costQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->costQuadraticApproximation(t, x, u);
  }

  ScalarFunctionQuadraticApproximation finalCostQuadraticApproximation(scalar_t t, const vector_t& x) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemCostsPtr_[activeMode]->finalCostQuadraticApproximation(t, x);
  }

 public:
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
  std::vector<std::shared_ptr<CostFunctionBase>> subsystemCostsPtr_{2};
  TargetTrajectories targetTrajectories_;
};

}  // namespace ocs2
