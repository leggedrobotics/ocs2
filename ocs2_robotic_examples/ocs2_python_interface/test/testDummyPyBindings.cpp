
#include <gtest/gtest.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include <ocs2_python_interface/PythonInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2 {
namespace pybindings_test {

class DummyInterface final : public RobotInterface {
 public:
  DummyInterface() {
    problemPtr_.reset(new OptimalControlProblem);

    matrix_t A(2, 2);
    A << 0, 1, 0, 0;
    matrix_t B(2, 1);
    B << 0, 1;
    problemPtr_->dynamicsPtr.reset(new LinearSystemDynamics(A, B));

    matrix_t Q(2, 2), R(1, 1), Qf(2, 2);
    Q << 1, 0, 0, 1;
    R << 1;
    Qf << 2, 0, 0, 2;
    problemPtr_->costPtr->add("cost", std::unique_ptr<StateInputCost>(new QuadraticStateInputCost(Q, R)));
    problemPtr_->finalCostPtr->add("finalCost", std::unique_ptr<StateCost>(new QuadraticStateCost(Qf)));

    costDesiredTrajectories_ = CostDesiredTrajectories({0.0}, {vector_t::Zero(2)}, {vector_t::Zero(2)});
    problemPtr_->costDesiredTrajectories = &costDesiredTrajectories_;

    operatingPointPtr_.reset(new OperatingPoints(vector_t::Zero(2), vector_t::Zero(1)));

    rollout::Settings rolloutSettings;
    rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings));
  }
  ~DummyInterface() override = default;

  std::unique_ptr<ocs2::MPC_DDP> getMpc() {
    mpc::Settings mpcSettings;
    ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = ddp::Algorithm::SLQ;
    return std::unique_ptr<MPC_DDP>(new MPC_DDP(mpcSettings, ddpSettings, *rolloutPtr_, *problemPtr_, *operatingPointPtr_));
  }

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
  const OperatingPoints& getOperatingPoints() const override { return *operatingPointPtr_; }

 private:
  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::unique_ptr<OperatingPoints> operatingPointPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  CostDesiredTrajectories costDesiredTrajectories_;
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
