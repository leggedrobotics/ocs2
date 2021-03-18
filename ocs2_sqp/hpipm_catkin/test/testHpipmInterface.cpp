/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "hpipm_catkin/HpipmInterface.h"

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

  // Solve again!
  hpipmInterface.resize(ocpSize);
  hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);

  // Initial condition
  ASSERT_TRUE(xSol[0].isApprox(x0));

  // Check dynamic feasibility
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] + system[k].dfdu * uSol[k] + system[k].f));
  }
}

TEST(test_hpiphm_interface, knownSolution) {
  int nx = 3;
  int nu = 2;
  int N = 5;

  // Problem setup.
  std::vector<ocs2::vector_t> xSolGiven;
  std::vector<ocs2::vector_t> uSolGiven;
  xSolGiven.emplace_back(ocs2::vector_t::Random(nx));
//  xSolGiven.emplace_back(ocs2::vector_t::Zero(nx));
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    // Pick optimal u
    uSolGiven.emplace_back(ocs2::vector_t::Random(nu));

    // Set optimal next x consistent with dynamics
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    xSolGiven.emplace_back(system[k].f + system[k].dfdx * xSolGiven[k] + system[k].dfdu * uSolGiven[k]);

    // Pick cost that minimizes at the given trajectory
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
    cost[k].dfdx = -(cost[k].dfdxx * xSolGiven[k] + cost[k].dfdux.transpose() * uSolGiven[k]);
    cost[k].dfdu = -(cost[k].dfduu * uSolGiven[k] + cost[k].dfdux * xSolGiven[k]);
  }
  cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, 0));
  cost[N].dfdx = -cost[N].dfdxx * xSolGiven[N];

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(xSolGiven[0], system, cost, nullptr, xSol, uSol, true);

  // Check!
  ASSERT_TRUE(ocs2::qp_solver::isEqual(xSolGiven, xSol, 1e-9));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(uSolGiven, uSol, 1e-9));
}
