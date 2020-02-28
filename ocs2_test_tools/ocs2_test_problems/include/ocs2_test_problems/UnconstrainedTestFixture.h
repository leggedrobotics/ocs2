//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <gtest/gtest.h>

#include "ocs2_test_problems/TestProblemSolution.h"
#include "ocs2_test_problems/UnconstrainedTestProblem.h"

template <typename TestDims>
class UnconstrainedTestFixture : public ::testing::Test {
  using testProblem_t = UnconstrainedTestProblem<TestDims::stateDim, TestDims::inputDim>;
  using testSolution_t = TestProblemSolution<TestDims::stateDim, TestDims::inputDim>;

 public:
  const TestDims dims_;
  const testProblem_t problem_;

  UnconstrainedTestFixture()
      : problem_(generateUnconstrainedTestProblem<TestDims::stateDim, TestDims::inputDim>()),
        tolerance_(1e-6),
        solutionStepSize_(1e-3),
        solution_(solveUnconstrainedTestProblem<TestDims::stateDim, TestDims::inputDim>(problem_, solutionStepSize_)) {}

  bool isSolutionCorrect(testSolution_t solution) { return isEqual(solution, solution_, tolerance_); };

 private:
  const double tolerance_;
  const double solutionStepSize_;
  const testSolution_t solution_;



};

/**
 * Test dimensions for the UnconstrainedTestFixture
 * @tparam STATE_DIM
 * @tparam INPUT_DIM
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
struct Dims {
  static constexpr size_t stateDim_ = STATE_DIM;
  static constexpr size_t inputDim_ = INPUT_DIM;
};

using UnconstrainedTestConfigurations = ::testing::Types<Dims<1, 2>, Dims<2, 3>, Dims<5, 10>>;

TYPED_TEST_CASE(UnconstrainedTestFixture, UnconstrainedTestConfigurations);