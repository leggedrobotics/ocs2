//
// Created by rgrandia on 15.03.21.
//

#include <gtest/gtest.h>

#include <ocs2_anymal_mpc/AnymalInterface.h>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

class TestAnymalModel : public ::testing::Test {
 public:
  TestAnymalModel() {
    const std::string configName("c_series");
    const std::string path(__FILE__);
    const std::string dir = path.substr(0, path.find_last_of("/"));
    const std::string configFolder = dir + "/../config/" + configName;

    // Get interface
    anymalInterface = anymal::getAnymalInterface(anymal::getUrdfString(anymal::AnymalModel::Camel), configFolder);

    problem = anymalInterface->getOptimalControlProblem();

    // Cost desired
    ocs2::scalar_t initTime = 0.0;
    ocs2::scalar_t finalTime = 1.0;
    const ocs2::vector_t state = anymalInterface->getInitialState();
    const ocs2::vector_t input = ocs2::vector_t::Zero(switched_model::INPUT_DIM);
    targetTrajectories = ocs2::TargetTrajectories{{initTime}, {state}, {input}};
    problem.targetTrajectoriesPtr = &targetTrajectories;

    // Initialize
    anymalInterface->getReferenceManagerPtr()->preSolverRun(initTime, finalTime, state);
  }

  std::unique_ptr<switched_model::QuadrupedInterface> anymalInterface;
  ocs2::OptimalControlProblem problem;
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
    problem.dynamicsPtr->linearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Dynamics " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";

  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Constraint + ocs2::Request::Approximation;
  problem.preComputationPtr->request(request, t, x, u);

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    problem.equalityConstraintPtr->getLinearApproximation(t, x, u, *problem.preComputationPtr);
  }
  timer.endTimer();
  std::cout << "Constraints state-input " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    ocs2::approximateCost(problem, t, x, u);
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
    problem.dynamicsPtr->linearApproximation(t, x, u);
  }
  timer.endTimer();
  std::cout << "Dynamics " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, precomputation) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Constraint + ocs2::Request::Approximation;

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    problem.preComputationPtr->request(request, t, x, u);
  }
  timer.endTimer();
  std::cout << "Precomputation " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, constraints_eq) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 1000000;

  constexpr auto request = ocs2::Request::Constraint + ocs2::Request::Approximation;
  problem.preComputationPtr->request(request, t, x, u);

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    problem.equalityConstraintPtr->getLinearApproximation(t, x, u, *problem.preComputationPtr);
  }
  timer.endTimer();
  std::cout << "Constraints state-input " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, cost) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Constraint + ocs2::Request::Approximation;
  problem.preComputationPtr->request(request, t, x, u);

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    ocs2::approximateCost(problem, t, x, u);
  }
  timer.endTimer();
  std::cout << "Cost " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}

TEST_F(TestAnymalModel, terminalCost) {
  ocs2::scalar_t t = 0.0;
  const ocs2::vector_t x = anymalInterface->getInitialState();
  const ocs2::vector_t u = ocs2::vector_t::Zero(24);
  ocs2::benchmark::RepeatedTimer timer;
  int N = 100000;

  constexpr auto request = ocs2::Request::Cost + ocs2::Request::SoftConstraint + ocs2::Request::Approximation;
  problem.preComputationPtr->requestFinal(request, t, x);

  timer.startTimer();
  for (int i = 0; i < N; i++) {
    ocs2::approximateFinalCost(problem, t, x);
  }
  timer.endTimer();
  std::cout << "Cost " << timer.getLastIntervalInMilliseconds() / N << " ms per call\n";
}
