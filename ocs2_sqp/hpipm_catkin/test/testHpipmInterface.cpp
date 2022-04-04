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

#include <ocs2_core/test/testTools.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

TEST(test_hpiphm_interface, solve_and_check_dynamic) {
  int nx = 3;
  int nu = 2;
  int N = 5;

  // Problem setup
  ocs2::vector_t x0 = ocs2::vector_t::Random(nx);
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  const auto status = hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

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
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));

  // Resize Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  hpipmInterface.resize(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);

  // Solve again!
  hpipmInterface.resize(ocpSize);
  const auto status = hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

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
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    // Pick optimal u
    uSolGiven.emplace_back(ocs2::vector_t::Random(nu));

    // Set optimal next x consistent with dynamics
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    xSolGiven.emplace_back(system[k].f + system[k].dfdx * xSolGiven[k] + system[k].dfdu * uSolGiven[k]);

    // Pick cost that minimizes at the given trajectory
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
    cost[k].dfdx = -(cost[k].dfdxx * xSolGiven[k] + cost[k].dfdux.transpose() * uSolGiven[k]);
    cost[k].dfdu = -(cost[k].dfduu * uSolGiven[k] + cost[k].dfdux * xSolGiven[k]);
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));
  cost[N].dfdx = -cost[N].dfdxx * xSolGiven[N];

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  const auto status = hpipmInterface.solve(xSolGiven[0], system, cost, nullptr, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

  // Check!
  ASSERT_TRUE(ocs2::isEqual(xSolGiven, xSol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(uSolGiven, uSol, 1e-9));
}

TEST(test_hpiphm_interface, with_constraints) {
  // Initialize without size
  ocs2::HpipmInterface hpipmInterface;

  int nx = 3;
  int nu = 2;
  int nc = 1;
  int N = 5;

  // Problem setup
  ocs2::vector_t x0 = ocs2::vector_t::Random(nx);
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::VectorFunctionLinearApproximation> constraints;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
    constraints.emplace_back(ocs2::getRandomConstraints(nx, nu, nc));
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));
  constraints.emplace_back(ocs2::getRandomConstraints(nx, 0, nc));

  // Resize Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  std::fill(ocpSize.numIneqConstraints.begin(), ocpSize.numIneqConstraints.end(), nc);

  // Set one of the constraints to empty
  constraints[1] = ocs2::VectorFunctionLinearApproximation();
  ocpSize.numIneqConstraints[1] = 0;

  hpipmInterface.resize(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  const auto status = hpipmInterface.solve(x0, system, cost, &constraints, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

  // Initial condition
  ASSERT_TRUE(xSol[0].isApprox(x0));

  // Check dynamic feasibility
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] + system[k].dfdu * uSol[k] + system[k].f, 1e-9));
  }

  // Check constraints
  for (int k = 0; k < N; k++) {
    if (constraints[k].f.size() > 0) {
      ASSERT_TRUE(constraints[k].f.isApprox(-constraints[k].dfdx * xSol[k] - constraints[k].dfdu * uSol[k], 1e-9));
    }
  }
}

TEST(test_hpiphm_interface, noInputs) {
  // Initialize without size
  ocs2::HpipmInterface hpipmInterface;

  int nx = 3;
  int nu;  // timeVarying
  int N = 5;

  // Problem setup.
  std::vector<ocs2::vector_t> xSolGiven;
  std::vector<ocs2::vector_t> uSolGiven;
  xSolGiven.emplace_back(ocs2::vector_t::Random(nx));
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    if (k == 1) {
      nu = 0;
    } else {
      nu = 2;
    }

    // Pick optimal u
    uSolGiven.emplace_back(ocs2::vector_t::Random(nu));

    // Set optimal next x consistent with dynamics
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    xSolGiven.emplace_back(system[k].f + system[k].dfdx * xSolGiven[k] + system[k].dfdu * uSolGiven[k]);

    // Pick cost that minimizes at the given trajectory
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
    cost[k].dfdx = -(cost[k].dfdxx * xSolGiven[k] + cost[k].dfdux.transpose() * uSolGiven[k]);
    cost[k].dfdu = -(cost[k].dfduu * uSolGiven[k] + cost[k].dfdux * xSolGiven[k]);
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));
  cost[N].dfdx = -cost[N].dfdxx * xSolGiven[N];

  const auto ocpSize = ocs2::hpipm_interface::extractSizesFromProblem(system, cost, nullptr);
  hpipmInterface.resize(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  const auto status = hpipmInterface.solve(xSolGiven.front(), system, cost, nullptr, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

  // Check!
  ASSERT_TRUE(ocs2::isEqual(xSolGiven, xSol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(uSolGiven, uSol, 1e-9));
}

TEST(test_hpiphm_interface, retrieveRiccati) {
  int nx = 3;
  int nu = 2;
  int N = 5;

  // Problem setup
  ocs2::vector_t x0 = ocs2::vector_t::Random(nx);
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    system.emplace_back(ocs2::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::getRandomCost(nx, 0));

  // store the true cost-to-go and feedback/feedforward terms
  std::vector<ocs2::matrix_t> SmSolGiven(N + 1);
  std::vector<ocs2::vector_t> svSolGiven(N + 1);
  std::vector<ocs2::scalar_t> s0SolGiven(N + 1, 0.0);  // Currently always returns zero, but we check that it exactly does.
  std::vector<ocs2::matrix_t> KSolGiven(N);
  std::vector<ocs2::vector_t> kSolGiven(N);

  // Discrete time Riccati recursion
  SmSolGiven[N] = cost[N].dfdxx;
  svSolGiven[N] = cost[N].dfdx;
  for (int k = N - 1; k >= 0; k--) {
    // Shorthand
    const auto& Sm = SmSolGiven[k + 1];
    const auto& sv = svSolGiven[k + 1];
    const auto& A = system[k].dfdx;
    const auto& B = system[k].dfdu;
    const auto& b = system[k].f;
    const auto& Q = cost[k].dfdxx;
    const auto& R = cost[k].dfduu;
    const auto& P = cost[k].dfdux;
    const auto& q = cost[k].dfdx;
    const auto& r = cost[k].dfdu;
    ocs2::matrix_t P_BTSmA = P + B.transpose() * Sm * A;
    ocs2::matrix_t invR_BTSmB = (R + B.transpose() * Sm * B).inverse();
    ocs2::vector_t r_BTsv_BTSmb = r + B.transpose() * sv + B.transpose() * Sm * b;

    // The actual backward pass
    SmSolGiven[k] = Q + A.transpose() * Sm * A - P_BTSmA.transpose() * invR_BTSmB * P_BTSmA;
    svSolGiven[k] = q + A.transpose() * sv + A.transpose() * Sm * b - P_BTSmA.transpose() * invR_BTSmB * r_BTsv_BTSmb;
    KSolGiven[k] = -invR_BTSmB * P_BTSmA;
    kSolGiven[k] = -invR_BTSmB * r_BTsv_BTSmb;
  }

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  const auto status = hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);
  ASSERT_EQ(status, hpipm_status::SUCCESS);

  // Get Riccati info from hpipm interface
  const auto KSol = hpipmInterface.getRiccatiFeedback(system[0], cost[0]);
  const auto kSol = hpipmInterface.getRiccatiFeedforward(system[0], cost[0]);
  const auto CostToGo = hpipmInterface.getRiccatiCostToGo(system[0], cost[0]);
  std::vector<ocs2::matrix_t> SmSol(N + 1);
  std::vector<ocs2::vector_t> svSol(N + 1);
  std::vector<ocs2::scalar_t> s0Sol(N + 1);
  for (int i = 0; i < (N + 1); i++) {
    SmSol[i] = CostToGo[i].dfdxx;
    svSol[i] = CostToGo[i].dfdx;
    s0Sol[i] = CostToGo[i].f;
  }

  // Compare the two vector/matrix trajectory
  ASSERT_TRUE(ocs2::isEqual(SmSolGiven, SmSol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(svSolGiven, svSol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(s0SolGiven, s0Sol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(KSolGiven, KSol, 1e-9));
  ASSERT_TRUE(ocs2::isEqual(kSolGiven, kSol, 1e-9));

  // Check self-consistency of returned elements in u = K * x + k
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(uSol[k].isApprox(KSol[k] * xSol[k] + kSol[k]));
  }
}
