//
// Created by rgrandia on 15.03.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h>

#include <ocs2_core/misc/Benchmark.h>

class TestAnymalLoopshapingModel : public ::testing::Test {
 public:
  TestAnymalLoopshapingModel() {
    const std::string robotName("chip");
    const std::string configName("c_series");
    const std::string path(__FILE__);
    const std::string dir = path.substr(0, path.find_last_of("/"));
    const std::string configFolder = dir + "/../config/" + configName;

    // Get interface
    anymalInterface = anymal::getAnymalLoopshapingInterface(anymal::stringToAnymalModel(robotName), configFolder);

    // Cost desired
    ocs2::scalar_t initTime = 0.0;
    ocs2::scalar_t finalTime = 1.0;
    const ocs2::vector_t systemState = anymalInterface->getInitialState().head(switched_model::STATE_DIM);
    const ocs2::vector_t state = anymalInterface->getInitialState();
    const ocs2::vector_t input = ocs2::vector_t::Zero(switched_model::INPUT_DIM);
    targetTrajectories = ocs2::TargetTrajectories{{initTime}, {systemState}, {input}};

    dynamics.reset(anymalInterface->getDynamics().clone());
    cost.reset(anymalInterface->getCost().clone());
    constraints.reset(anymalInterface->getConstraintPtr()->clone());

    // Initialize
    anymalInterface->getReferenceManagerPtr()->preSolverRun(initTime, finalTime, state);
    cost->setTargetTrajectoriesPtr(&targetTrajectories);
  }

  std::unique_ptr<switched_model_loopshaping::QuadrupedLoopshapingInterface> anymalInterface;
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamics;
  std::unique_ptr<ocs2::CostFunctionBase> cost;
  std::unique_ptr<ocs2::ConstraintBase> constraints;
  ocs2::TargetTrajectories targetTrajectories;
};

TEST_F(TestAnymalLoopshapingModel, all) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < 4*N; i++) {
    dynamics->linearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Dynamics " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    constraints->stateInputEqualityConstraintLinearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Constraints state-input " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    constraints->inequalityConstraintQuadraticApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Constraints inequality " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    cost->costQuadraticApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Cost " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalLoopshapingModel, dynamics) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < 4*N; i++) {
    dynamics->linearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Dynamics " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalLoopshapingModel, constraints_eq) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    constraints->stateInputEqualityConstraintLinearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Constraints equality " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalLoopshapingModel, constraints_ineq) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    constraints->inequalityConstraintQuadraticApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Constraints inequality " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalLoopshapingModel, cost) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    cost->costQuadraticApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Cost " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}
