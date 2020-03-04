//
// Created by rgrandia on 26.02.20.
//

#include <gtest/gtest.h>

#include "ocs2_qp_solver/wrappers/CostWrapper.h"

#include "testProblemsGeneration.h"

class CostWrapperTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 3;
  static constexpr size_t INPUT_DIM = 2;
  using costFunction_t = ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>;
  using scalar_t = costFunction_t::scalar_t;
  using input_vector_t = costFunction_t::input_vector_t;
  using state_vector_t = costFunction_t::state_vector_t;

  CostWrapperTest() {
    // Construct random cost matrices
    srand(0);
    cost = ocs2::qp_solver::getOcs2Cost<STATE_DIM, INPUT_DIM>(ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                              ocs2::qp_solver::getRandomCost(STATE_DIM, INPUT_DIM),
                                                              state_vector_t::Random(), input_vector_t::Random(), state_vector_t::Random());
    costWrapper.reset(new ocs2::qp_solver::CostWrapper(*cost));

    // Setpoint
    t = 0.42;
    x = state_vector_t::Random();
    u = input_vector_t::Random();

    // Prepare cost at setpoint after wrapping
    cost->setCurrentStateAndControl(t, x, u);
  }

  double t;
  state_vector_t x;
  input_vector_t u;
  std::unique_ptr<ocs2::qp_solver::CostWrapper> costWrapper;
  std::unique_ptr<costFunction_t> cost;
};

TEST_F(CostWrapperTest, intermediateCostValue) {
  scalar_t L;
  cost->getIntermediateCost(L);
  ASSERT_DOUBLE_EQ(L, costWrapper->getCost(t, x, u));
}

TEST_F(CostWrapperTest, intermediateCostValueAfterCopy) {
  scalar_t L;
  cost->getIntermediateCost(L);
  cost.reset();
  // Copy and destroy old wrapper
  auto wrapperClone = *costWrapper;
  costWrapper.reset();
  ASSERT_DOUBLE_EQ(L, wrapperClone.getCost(t, x, u));
}

TEST_F(CostWrapperTest, intermediateQuadraticApproximation) {
  // Define deviation
  double dt = 0.24;
  state_vector_t dx = state_vector_t::Random();
  input_vector_t du = input_vector_t::Random();

  // Cost at deviation
  scalar_t L_true;
  cost->setCurrentStateAndControl(t + dt, x + dx, u + du);
  cost->getIntermediateCost(L_true);
  cost.reset();  // Destroy the cost function after evaluation

  const auto quadraticApproximation = costWrapper->getQuadraticApproximation(t, x, u);
  const auto& dfdxx = quadraticApproximation.dfdxx;
  const auto& dfdux = quadraticApproximation.dfdux;
  const auto& dfduu = quadraticApproximation.dfduu;
  const auto& dfdx = quadraticApproximation.dfdx;
  const auto& dfdu = quadraticApproximation.dfdu;
  const auto& l = quadraticApproximation.f;
  auto L_wrapped_approximation = 0.5 * dx.dot(dfdxx * dx) + 0.5 * du.dot(dfduu * du) + du.dot(dfdux * dx) + dfdx.dot(dx) + dfdu.dot(du) + l;

  ASSERT_DOUBLE_EQ(L_true, L_wrapped_approximation);
}

TEST_F(CostWrapperTest, terminalCostValue) {
  scalar_t L;
  cost->getTerminalCost(L);
  ASSERT_DOUBLE_EQ(L, costWrapper->getTerminalCost(t, x));
}

TEST_F(CostWrapperTest, terminalQuadraticApproximation) {
  // Define deviation
  double dt = 0.24;
  state_vector_t dx = state_vector_t::Random();

  // Cost at deviation
  scalar_t L_true;
  cost->setCurrentStateAndControl(t + dt, x + dx, u);
  cost->getTerminalCost(L_true);
  cost.reset();  // Destroy the cost function after evaluation

  const auto quadraticApproximation = costWrapper->getTerminalQuadraticApproximation(t, x);
  const auto& dfdxx = quadraticApproximation.dfdxx;
  const auto& dfdx = quadraticApproximation.dfdx;
  const auto& l = quadraticApproximation.f;
  auto L_wrapped_approximation = 0.5 * dx.dot(dfdxx * dx) + dfdx.dot(dx) + l;

  ASSERT_DOUBLE_EQ(L_true, L_wrapped_approximation);
}
