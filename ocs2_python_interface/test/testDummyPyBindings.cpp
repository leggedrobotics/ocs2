
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
    problem_.costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
    problem_.finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

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
    return std::unique_ptr<GaussNewtonDDP_MPC>(new GaussNewtonDDP_MPC(mpcSettings, ddpSettings, *rolloutPtr_, problem_, *initializerPtr_));
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
