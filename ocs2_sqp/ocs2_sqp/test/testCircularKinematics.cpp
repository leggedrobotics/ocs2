//
// Created by rgrandia on 09.03.21.
//

#include <gtest/gtest.h>

#include "ocs2_sqp/MultipleShootingSolver.h"

#include <ocs2_oc/test/circular_kinematics.h>

TEST(test_circular_kinematics, solve_projected_EqConstraints) {
  ocs2::CircularKinematicsSystem system;
  ocs2::CircularKinematicsCost cost;
  cost.initialize("circular_kinematics_cost", "/tmp/ocs2", true, false);
  ocs2::CircularKinematicsConstraints constraint;

  // Solver settings
  ocs2::MultipleShootingSolverSettings settings;
  settings.dt = 0.01;
  settings.n_state = 2;
  settings.n_input = 2;
  settings.sqpIteration = 20;
  settings.qr_decomp = true;
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  ocs2::MultipleShootingSolver solver(settings, &system, &cost, &constraint);

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0
  const ocs2::scalar_array_t partitioningTimes{0.0};                            // doesn't matter

  // Solve
  solver.run(startTime, initState, finalTime, partitioningTimes);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.stateEqConstraintISE, 1e-6);
  ASSERT_LT(performance.stateInputEqConstraintISE, 1e-6);
}

TEST(test_circular_kinematics, solve_EqConstraints_inQPSubproblem) {
  ocs2::CircularKinematicsSystem system;
  ocs2::CircularKinematicsCost cost;
  cost.initialize("circular_kinematics_cost", "/tmp/ocs2", true, false);
  ocs2::CircularKinematicsConstraints constraint;

  // Solver settings
  ocs2::MultipleShootingSolverSettings settings;
  settings.dt = 0.01;
  settings.n_state = 2;
  settings.n_input = 2;
  settings.sqpIteration = 20;
  settings.qr_decomp = false; // <- false to turn off projection of state-input equalities
  settings.printSolverStatistics = true;
  settings.printSolverStatus = true;
  settings.printLinesearch = true;
  ocs2::MultipleShootingSolver solver(settings, &system, &cost, &constraint);

  // Additional problem definitions
  const ocs2::scalar_t startTime = 0.0;
  const ocs2::scalar_t finalTime = 1.0;
  const ocs2::vector_t initState = (ocs2::vector_t(2) << 1.0, 0.0).finished();  // radius 1.0
  const ocs2::scalar_array_t partitioningTimes{0.0};                            // doesn't matter

  // Solve
  solver.run(startTime, initState, finalTime, partitioningTimes);

  // Inspect solution
  const auto primalSolution = solver.primalSolution(finalTime);
  for (int i = 0; i < primalSolution.timeTrajectory_.size(); i++) {
    std::cout << "time: " << primalSolution.timeTrajectory_[i] << "\t state: " << primalSolution.stateTrajectory_[i].transpose()
              << "\t input: " << primalSolution.inputTrajectory_[i].transpose() << std::endl;
  }

  // Check initial condition
  ASSERT_TRUE(primalSolution.stateTrajectory_.front().isApprox(initState));
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.front(), startTime);
  ASSERT_DOUBLE_EQ(primalSolution.timeTrajectory_.back(), finalTime);

  // Check constraint satisfaction.
  const auto performance = solver.getPerformanceIndeces();
  ASSERT_LT(performance.stateEqConstraintISE, 1e-6);
  ASSERT_LT(performance.stateInputEqConstraintISE, 1e-6);
}