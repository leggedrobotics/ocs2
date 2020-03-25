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
