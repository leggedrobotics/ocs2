//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include "ocs2_test_problems/TestProblemSolution.h"



template <size_t STATE_DIM, size_t INPUT_DIM>
struct UnconstrainedTestProblem {
  std::pair<double, double> timeHorizon;
  std::unique_ptr<ocs2::CostFunctionBase<STATE_DIM, INPUT_DIM>> costFunction_;
  std::unique_ptr<ocs2::SystemDynamicsBase<STATE_DIM, INPUT_DIM>> systemDynamics_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
UnconstrainedTestProblem<STATE_DIM, INPUT_DIM> generateUnconstrainedTestProblem();

template <size_t STATE_DIM, size_t INPUT_DIM>
TestProblemSolution<STATE_DIM, INPUT_DIM> solveUnconstrainedTestProblem(UnconstrainedTestProblem<STATE_DIM, INPUT_DIM> problem);