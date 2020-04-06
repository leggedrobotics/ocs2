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

#include "ocs2_qp_solver/test/testProblemsGeneration.h"
#include "ocs2_qp_solver/wrappers/ConstraintsWrapper.h"

class ConstraintsWrapperTest : public testing::Test {
 protected:
  static constexpr size_t STATE_DIM = 8;
  static constexpr size_t INPUT_DIM = 4;
  static constexpr size_t stateInputConstraints = 3;
  static constexpr size_t stateOnlyConstraints = 2;
  static constexpr size_t finalStateOnlyConstraints = 3;

  using constraintFunction_t = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;
  using scalar_t = constraintFunction_t::scalar_t;
  using input_vector_t = constraintFunction_t::input_vector_t;
  using state_vector_t = constraintFunction_t::state_vector_t;

  ConstraintsWrapperTest() {
    // Construct random constraint matrices
    srand(0);
    constraint = ocs2::qp_solver::getOcs2Constraints<STATE_DIM, INPUT_DIM>(
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, stateInputConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, stateOnlyConstraints),
        ocs2::qp_solver::getRandomConstraints(STATE_DIM, INPUT_DIM, finalStateOnlyConstraints));
    constraintsWrapper.reset(new ocs2::qp_solver::ConstraintsWrapper(*constraint));

    // Setpoint
    t = 0.42;
    x = state_vector_t::Random();
    u = input_vector_t::Random();

    // Prepare constraint at setpoint after wrapping
    constraint->setCurrentStateAndControl(t, x, u);
  }

  scalar_t t;
  state_vector_t x;
  input_vector_t u;
  std::unique_ptr<ocs2::qp_solver::ConstraintsWrapper> constraintsWrapper;
  std::unique_ptr<constraintFunction_t> constraint;
};

constexpr size_t ConstraintsWrapperTest::STATE_DIM;
constexpr size_t ConstraintsWrapperTest::INPUT_DIM;
constexpr size_t ConstraintsWrapperTest::stateInputConstraints;
constexpr size_t ConstraintsWrapperTest::stateOnlyConstraints;
constexpr size_t ConstraintsWrapperTest::finalStateOnlyConstraints;

TEST_F(ConstraintsWrapperTest, intermediateConstraintValue) {
  constraintFunction_t::constraint1_vector_t e;
  constraint->getConstraint1(e);
  const auto nc1 = constraint->numStateInputConstraint(t);
  constraintFunction_t::constraint2_vector_t h;
  constraint->getConstraint2(h);
  const auto nc2 = constraint->numStateOnlyConstraint(t);
  ocs2::dynamic_vector_t g(nc1 + nc2);
  g << e.head(nc1), h.head(nc2);
  ASSERT_TRUE(g.isApprox(constraintsWrapper->getConstraint(t, x, u)));
}

TEST_F(ConstraintsWrapperTest, intermediateConstraintValueAfterCopy) {
  const auto g = constraintsWrapper->getConstraint(t, x, u);
  // Copy and destroy old wrapper
  auto wrapperClone = *constraintsWrapper;
  constraintsWrapper.reset();
  ASSERT_TRUE(g.isApprox(wrapperClone.getConstraint(t, x, u)));
}

TEST_F(ConstraintsWrapperTest, intermediateLinearApproximation) {
  // Define deviation
  scalar_t dt = 0.24;
  state_vector_t dx = state_vector_t::Random();
  input_vector_t du = input_vector_t::Random();

  // Constraint at deviation
  constraint->setCurrentStateAndControl(t + dt, x + dx, u + du);
  constraintFunction_t::constraint1_vector_t e_true;
  constraint->getConstraint1(e_true);
  const auto nc1 = constraint->numStateInputConstraint(t + dt);
  constraintFunction_t::constraint2_vector_t h_true;
  constraint->getConstraint2(h_true);
  const auto nc2 = constraint->numStateOnlyConstraint(t + dt);
  ocs2::dynamic_vector_t g_true(nc1 + nc2);
  g_true << e_true.head(nc1), h_true.head(nc2);
  constraint.reset();  // Destroy the constraint function after evaluation

  const auto linearApproximation = constraintsWrapper->getLinearApproximation(t, x, u);
  ocs2::dynamic_vector_t g_wrapped_approximation = linearApproximation.f + linearApproximation.dfdx * dx + linearApproximation.dfdu * du;

  ASSERT_TRUE(g_true.isApprox(g_wrapped_approximation));
}

TEST_F(ConstraintsWrapperTest, terminalConstraintValue) {
  constraintFunction_t::constraint2_vector_t hfTemp;
  constraint->getFinalConstraint2(hfTemp);
  ocs2::dynamic_vector_t hf = hfTemp.head(constraint->numStateOnlyFinalConstraint(t));
  const auto nc = constraint->numStateOnlyFinalConstraint(t);
  ASSERT_TRUE(hfTemp.head(nc).isApprox(constraintsWrapper->getTerminalConstraint(t, x)));
}

TEST_F(ConstraintsWrapperTest, terminalLinearApproximation) {
  // Define deviation
  scalar_t dt = 0.24;
  state_vector_t dx = state_vector_t::Random();

  // Constraint at deviation
  constraint->setCurrentStateAndControl(t + dt, x + dx, u);
  constraintFunction_t::constraint2_vector_t h_true;
  constraint->getFinalConstraint2(h_true);
  const auto nc = constraint->numStateOnlyFinalConstraint(t + dt);
  constraint.reset();  // Destroy the constraint function after evaluation

  const auto linearApproximation = constraintsWrapper->getTerminalLinearApproximation(t, x);
  ocs2::dynamic_vector_t g_wrapped_approximation = linearApproximation.f + linearApproximation.dfdx * dx;

  ASSERT_TRUE(h_true.head(nc).isApprox(g_wrapped_approximation));
}
