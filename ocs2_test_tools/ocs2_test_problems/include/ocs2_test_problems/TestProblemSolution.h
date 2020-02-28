//
// Created by rgrandia on 25.02.20.
//

#pragma once

#include <Eigen/Dense>
#include <vector>

template <size_t STATE_DIM, size_t INPUT_DIM>
struct TestProblemSolution {
  std::vector<double> timeTrajectory_;
  std::vector<Eigen::Matrix<double, STATE_DIM, 1>> stateTrajectory_;
  std::vector<Eigen::Matrix<double, INPUT_DIM, 1>> inputTrajectory_;
};

template <size_t STATE_DIM, size_t INPUT_DIM>
bool isEqual(TestProblemSolution<STATE_DIM, INPUT_DIM> solutionA, TestProblemSolution<STATE_DIM, INPUT_DIM> solutionB, double tolerance) {
  return true;
}