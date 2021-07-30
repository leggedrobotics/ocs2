//
// Created by rgrandia on 15.03.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_mpc/AnymalInterface.h>

#include <ocs2_core/misc/Benchmark.h>

class TestAnymalModel : public ::testing::Test {
 public:
  TestAnymalModel() {
    const std::string robotName("croc");
    const std::string configName("c_series");
    const std::string path(__FILE__);
    const std::string dir = path.substr(0, path.find_last_of("/"));
    const std::string configFolder = dir + "/../config/" + configName;

    // Get interface
    anymalInterface = anymal::getAnymalInterface(anymal::stringToAnymalModel(robotName), configFolder);

    // Cost desired
    ocs2::scalar_t initTime = 0.0;
    ocs2::scalar_t finalTime = 1.0;
    const ocs2::vector_t state = anymalInterface->getInitialState();
    const ocs2::vector_t input = ocs2::vector_t::Zero(switched_model::INPUT_DIM);
    targetTrajectories = ocs2::TargetTrajectories{{initTime}, {state}, {input}};

    dynamics.reset(anymalInterface->getDynamics().clone());
    cost.reset(anymalInterface->getCost().clone());
    constraints.reset(anymalInterface->getConstraintPtr()->clone());

    // Initialize
    anymalInterface->getReferenceManagerPtr()->preSolverRun(initTime, finalTime, state);
    cost->setTargetTrajectoriesPtr(&targetTrajectories);
  }

  std::unique_ptr<switched_model::QuadrupedInterface> anymalInterface;
  std::unique_ptr<ocs2::SystemDynamicsBase> dynamics;
  std::unique_ptr<ocs2::CostFunctionBase> cost;
  std::unique_ptr<ocs2::ConstraintBase> constraints;
  ocs2::TargetTrajectories targetTrajectories;
};

TEST_F(TestAnymalModel, all) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
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

TEST_F(TestAnymalModel, dynamics) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    dynamics->linearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Dynamics " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, constraints_eq) {
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
  std::cout << "Constraints state-input " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, constraints_ineq) {
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

TEST_F(TestAnymalModel, cost) {
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
