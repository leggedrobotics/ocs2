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

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <ocs2_oc/synchronized_module/ModeScheduleManager.h>

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

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->computeFlowMap(t, x, u, preComp);
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) final {
    const auto activeMode = modeScheduleManagerPtr_->getModeSchedule().modeAtTime(t);
    return subsystemDynamicsPtr_[activeMode]->linearApproximation(t, x, u, preComp);
  }

 public:
  std::vector<std::shared_ptr<SystemDynamicsBase>> subsystemDynamicsPtr_;
  std::shared_ptr<ModeScheduleManager> modeScheduleManagerPtr_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_Cost final : public QuadraticStateInputCost {
 public:
  EXP0_Cost() : QuadraticStateInputCost((matrix_t(2, 2) << 0.0, 0.0, 0.0, 1.0).finished(), matrix_t::Identity(1, 1)) {
    xDesired_ = (vector_t(2) << 4.0, 2.0).finished();
    uDesired_ = (vector_t(1) << 0.0).finished();
  }
  ~EXP0_Cost() override = default;
  EXP0_Cost* clone() const override { return new EXP0_Cost(*this); }

 protected:
  std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t& state, const vector_t& input,
                                                       const CostDesiredTrajectories& desiredTrajectory) const override {
    return {state - xDesired_, input - uDesired_};
  }

 public:
  vector_t xDesired_;
  vector_t uDesired_;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
class EXP0_FinalCost final : public QuadraticStateCost {
 public:
  EXP0_FinalCost() : QuadraticStateCost(matrix_t::Identity(2, 2)), xDesired_((vector_t(2) << 4.0, 2.0).finished()) {}
  ~EXP0_FinalCost() override = default;
  EXP0_FinalCost* clone() const override { return new EXP0_FinalCost(*this); }

 protected:
  vector_t getStateDeviation(scalar_t time, const vector_t& state, const CostDesiredTrajectories& desiredTrajectory) const override {
    return state - xDesired_;
  }

 public:
  vector_t xDesired_;
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
