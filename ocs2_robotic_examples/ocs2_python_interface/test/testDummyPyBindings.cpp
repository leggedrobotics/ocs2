
#include <gtest/gtest.h>

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_mpc/MPC_DDP.h>
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
    dynamicsPtr_.reset(new LinearSystemDynamics(A, B));

    const matrix_t Q = (matrix_t(2, 2) << 1, 0, 0, 1).finished();
    const matrix_t R = (matrix_t(1, 1) << 1).finished();
    const matrix_t Qf = (matrix_t(2, 2) << 2, 0, 0, 2).finished();
    costPtr_.reset(new QuadraticCostFunction(Q, R, Qf));

    targetTrajectories_ = TargetTrajectories({0.0}, {vector_t::Zero(2)}, {vector_t::Zero(1)});
    costPtr_->setTargetTrajectoriesPtr(&targetTrajectories_);

    constraintPtr_.reset(new ConstraintBase());

    initializerPtr_.reset(new DefaultInitializer(1));

    rollout::Settings rolloutSettings;
    rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings));
  }
  ~DummyInterface() override = default;

  std::unique_ptr<ocs2::MPC_DDP> getMpc() {
    ddp::Settings ddpSettings;
    ddpSettings.algorithm_ = ddp::Algorithm::SLQ;
    mpc::Settings mpcSettings;
    return std::unique_ptr<MPC_DDP>(new MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintPtr_.get(), costPtr_.get(),
                                                initializerPtr_.get(), ddpSettings, mpcSettings));
  }

  const SystemDynamicsBase& getDynamics() const override { return *dynamicsPtr_; }
  const CostFunctionBase& getCost() const override { return *costPtr_; }
  const ConstraintBase* getConstraintPtr() const override { return constraintPtr_.get(); }
  const Initializer& getInitializer() const override { return *initializerPtr_; }

  std::unique_ptr<LinearSystemDynamics> dynamicsPtr_;
  std::unique_ptr<QuadraticCostFunction> costPtr_;
  std::unique_ptr<ConstraintBase> constraintPtr_;
  std::unique_ptr<Initializer> initializerPtr_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  TargetTrajectories targetTrajectories_;
};

class DummyPyBindings final : public PythonInterface {
 public:
  using Base = PythonInterface;

  DummyPyBindings(const std::string& taskFileFolder) {
    DummyInterface robot;
    PythonInterface::init(robot, robot.getMpc());
  }
};

}  // namespace pybindings_test
}  // namespace ocs2

TEST(OCS2PyBindingsTest, createDummyPyBindings) {
  ocs2::pybindings_test::DummyPyBindings dummy("mpc");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
