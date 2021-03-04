//
// Created by rgrandia on 18.02.21.
//

#include <gtest/gtest.h>

#include "ocs2_sqp/HpipmInterface.h"

#include <ocs2_qp_solver/test/testProblemsGeneration.h>

TEST(test_hpiphm_interface, solve_and_check_dynamic) {
  int nx = 3;
  int nu = 2;
  int N = 5;

  // Problem setup
  ocs2::vector_t x0 = ocs2::vector_t::Random(nx);
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, 0));

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);

  // Initial condition
  ASSERT_TRUE(xSol[0].isApprox(x0));

  // Check dynamic feasibility
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] + system[k].dfdu * uSol[k] + system[k].f));
  }
}

TEST(test_hpiphm_interface, solve_after_resize) {
  // Initialize without size
  ocs2::HpipmInterface hpipmInterface;

  int nx = 3;
  int nu = 2;
  int N = 5;

  // Problem setup
  ocs2::vector_t x0 = ocs2::vector_t::Random(nx);
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, 0));

  // Resize Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  hpipmInterface.resize(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);

  // Initial condition
  ASSERT_TRUE(xSol[0].isApprox(x0));

  // Check dynamic feasibility
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] + system[k].dfdu * uSol[k] + system[k].f));
  }
}

