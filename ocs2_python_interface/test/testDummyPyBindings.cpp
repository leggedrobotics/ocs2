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

#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_python_interface/PythonInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2 {
namespace pybindings_test {

class DummyInterface final : public RobotInterface {
 public:
  DummyInterface() {
    const matrix_t A = (matrix_t(2, 2) << 0, 1, 0, 0).finished();
    const matrix_t B = (matrix_t(2, 1) << 0, 1).finished();
    problem_.dynamicsPtr.reset(new LinearSystemDynamics(A, B));

    const matrix_t Q = (matrix_t(2, 2) << 1, 0, 0, 1).finished();
    const matrix_t R = (matrix_t(1, 1) << 1).finished();
    const matrix_t Qf = (matrix_t(2, 2) << 2, 0, 0, 2).finished();
    problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
    problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

    problem_.targetTrajectoriesPtr = &targetTrajectories_;

    initializerPtr_.reset(new DefaultInitializer(1));

    rollout::Settings rolloutSettings;
    rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));
  }
  ~DummyInterface() override = default;

  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> getMpc() {
    mpc::Settings mpcSettings;
    ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = ddp::Algorithm::SLQ;
    return std::make_unique<GaussNewtonDDP_MPC>(std::move(mpcSettings), std::move(ddpSettings), *rolloutPtr_, problem_, *initializerPtr_);
  }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }
  const Initializer& getInitializer() const override { return *initializerPtr_; }

 private:
  OptimalControlProblem problem_;
  std::unique_ptr<Initializer> initializerPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  const TargetTrajectories targetTrajectories_ = TargetTrajectories({0.0}, {vector_t::Zero(2)}, {vector_t::Zero(2)});
};

class DummyPyBindings final : public PythonInterface {
 public:
  using Base = PythonInterface;

  DummyPyBindings() {
    DummyInterface robot;
    PythonInterface::init(robot, robot.getMpc());
  }
};

}  // namespace pybindings_test
}  // namespace ocs2

TEST(OCS2PyBindingsTest, createDummyPyBindings) {
  ocs2::pybindings_test::DummyPyBindings dummy;
}
