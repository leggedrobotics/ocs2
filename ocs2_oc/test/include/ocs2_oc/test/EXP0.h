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
class EXP0_Sys1 final : public LinearSystemDynamics {
 public:
  EXP0_Sys1() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 0.6, 1.2, -0.8, 3.4;
    LinearSystemDynamics::B_ << 1, 1;
  };
  ~EXP0_Sys1() = default;

  EXP0_Sys1* clone() const final { return new EXP0_Sys1(*this); }

 private:
  EXP0_Sys1(const EXP0_Sys1& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Sys2 final : public LinearSystemDynamics {
 public:
  EXP0_Sys2() : LinearSystemDynamics(matrix_t(2, 2), matrix_t(2, 1)) {
    LinearSystemDynamics::A_ << 4, 3, -1, 0;
    LinearSystemDynamics::B_ << 2, -1;
  };
  ~EXP0_Sys2() = default;

  EXP0_Sys2* clone() const final { return new EXP0_Sys2(*this); }

 private:
  EXP0_Sys2(const EXP0_Sys2& other) = default;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_System final : public SystemDynamicsBase {
 public:
  explicit EXP0_System(std::shared_ptr<ReferenceManager> referenceManagerPtr)
      : referenceManagerPtr_(std::move(referenceManagerPtr)), subsystemDynamicsPtr_(2) {
    subsystemDynamicsPtr_[0].reset(new EXP0_Sys1);
    subsystemDynamicsPtr_[1].reset(new EXP0_Sys2);
  }

  ~EXP0_System() override = default;

  EXP0_System* clone() const final { return new EXP0_System(*this); }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u);
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u) final {
    const auto activeMode = referenceManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->linearApproximation(t, x, u);
  }

 private:
  EXP0_System(const EXP0_System& other) : EXP0_System(other.referenceManagerPtr_) {}

  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_;
  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_CostFunction : public CostFunctionBase {
 public:
  explicit EXP0_CostFunction(std::shared_ptr<ReferenceManager> referenceManagerPtr) : referenceManagerPtr_(std::move(referenceManagerPtr)) {
    const matrix_t Q = (matrix_t(2, 2) << 0.0, 0.0, 0.0, 1.0).finished();
    const matrix_t R = (matrix_t(1, 1) << 1.0).finished();
    const matrix_t Qf = (matrix_t(2, 2) << 1.0, 0.0, 0.0, 1.0).finished();
    subsystemCostsPtr_[0].reset(new QuadraticCostFunction(Q, R, matrix_t::Zero(2, 2)));
    subsystemCostsPtr_[1].reset(new QuadraticCostFunction(Q, R, Qf));

    subsystemCostsPtr_[0]->setTargetTrajectoriesPtr(&referenceManagerPtr_->getTargetTrajectories());
    subsystemCostsPtr_[1]->setTargetTrajectoriesPtr(&referenceManagerPtr_->getTargetTrajectories());
  }

  ~EXP0_CostFunction() = default;

  EXP0_CostFunction* clone() const final { return new EXP0_CostFunction(*this); }

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
  EXP0_CostFunction(const EXP0_CostFunction& other) : EXP0_CostFunction(other.referenceManagerPtr_) {}

  std::shared_ptr<ReferenceManager> referenceManagerPtr_;
  std::vector<std::shared_ptr<CostFunctionBase>> subsystemCostsPtr_{2};
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::shared_ptr<ReferenceManager> getExp0ReferenceManager(scalar_array_t eventTimes, std::vector<size_t> modeSequence) {
  const vector_t x = (vector_t(2) << 4.0, 2.0).finished();
  const vector_t u = (vector_t(1) << 0.0).finished();
  TargetTrajectories targetTrajectories({0.0}, {x}, {u});

  ModeSchedule modeSchedule(std::move(eventTimes), std::move(modeSequence));

  return std::make_shared<ReferenceManager>(std::move(targetTrajectories), std::move(modeSchedule));
}

}  // namespace ocs2
