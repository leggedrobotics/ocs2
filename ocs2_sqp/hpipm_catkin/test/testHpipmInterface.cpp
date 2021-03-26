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
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] +
                                     system[k].dfdu * uSol[k] + system[k].f));
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
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] +
                                     system[k].dfdu * uSol[k] + system[k].f));
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
<<<<<<< HEAD
  //  xSolGiven.emplace_back(ocs2::vector_t::Zero(nx));
=======
>>>>>>> master
  std::vector<ocs2::VectorFunctionLinearApproximation> system;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> cost;
  for (int k = 0; k < N; k++) {
    // Pick optimal u
    uSolGiven.emplace_back(ocs2::vector_t::Random(nu));

    // Set optimal next x consistent with dynamics
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    xSolGiven.emplace_back(system[k].f + system[k].dfdx * xSolGiven[k] +
                           system[k].dfdu * uSolGiven[k]);

    // Pick cost that minimizes at the given trajectory
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
    cost[k].dfdx = -(cost[k].dfdxx * xSolGiven[k] +
                     cost[k].dfdux.transpose() * uSolGiven[k]);
    cost[k].dfdu =
        -(cost[k].dfduu * uSolGiven[k] + cost[k].dfdux * xSolGiven[k]);
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
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
    constraints.emplace_back(ocs2::qp_solver::getRandomConstraints(nx, nu, nc));
  }
  cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, 0));
  constraints.emplace_back(ocs2::qp_solver::getRandomConstraints(nx, 0, nc));

  // Resize Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  std::fill(ocpSize.numIneqConstraints.begin(),
            ocpSize.numIneqConstraints.end(), nc);
  hpipmInterface.resize(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(x0, system, cost, &constraints, xSol, uSol, true);

  // Initial condition
  ASSERT_TRUE(xSol[0].isApprox(x0));

  // Check dynamic feasibility
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(xSol[k + 1].isApprox(system[k].dfdx * xSol[k] +
                                         system[k].dfdu * uSol[k] + system[k].f,
                                     1e-9));
  }

  // Check constraints
  for (int k = 0; k < N; k++) {
    ASSERT_TRUE(constraints[k].f.isApprox(
        -constraints[k].dfdx * xSol[k] - constraints[k].dfdu * uSol[k], 1e-9));
  }
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
    system.emplace_back(ocs2::qp_solver::getRandomDynamics(nx, nu));
    cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, nu));
  }
  cost.emplace_back(ocs2::qp_solver::getRandomCost(nx, 0));

  // store the true cost-to-go and feedback/feedforward terms
  std::vector<ocs2::matrix_t> PSolGiven;
  std::vector<ocs2::matrix_t> KSolGiven;
  std::vector<ocs2::vector_t> pSolGiven;
  std::vector<ocs2::vector_t> kSolGiven;
  // std::vector<double> cSolGiven;
  PSolGiven.resize(N + 1);
  KSolGiven.resize(N);
  pSolGiven.resize(N + 1);
  kSolGiven.resize(N);
  // cSolGiven.resize(N + 1);

  PSolGiven[N] = cost[N].dfdxx;
  pSolGiven[N] = cost[N].dfdx;
  // cSolGiven[N] = 0;
  for (int k = N - 1; k >= 0; k--) {
    ocs2::matrix_t Ak = system[k].dfdx;
    ocs2::matrix_t Bk = system[k].dfdu;
    ocs2::matrix_t bk = system[k].f;
    ocs2::matrix_t Qk = cost[k].dfdxx;
    ocs2::matrix_t Rk = cost[k].dfduu;
    ocs2::matrix_t Sk = cost[k].dfdux;
    ocs2::vector_t qk = cost[k].dfdx;
    ocs2::vector_t rk = cost[k].dfdu;

    PSolGiven[k] = Qk + Ak.transpose() * PSolGiven[k + 1] * Ak -
                   (Sk.transpose() + Ak.transpose() * PSolGiven[k + 1] * Bk) *
                       (Rk + Bk.transpose() * PSolGiven[k + 1] * Bk).inverse() *
                       (Sk + Bk.transpose() * PSolGiven[k + 1] * Ak);
    pSolGiven[k] = qk + Ak.transpose() * pSolGiven[k + 1] +
                   Ak.transpose() * PSolGiven[k + 1] * bk -
                   (Sk.transpose() + Ak.transpose() * PSolGiven[k + 1] * Bk) *
                       (Rk + Bk.transpose() * PSolGiven[k + 1] * Bk).inverse() *
                       (rk + Bk.transpose() * pSolGiven[k + 1] +
                        Bk.transpose() * PSolGiven[k + 1] * bk);
    // cSolGiven[k] = cSolGiven[k + 1] + bk.transpose() * pSolGiven[k + 1] + 0.5
    // * bk.transpose() * PSolGiven[k + 1] * bk -
    //                0.5 * (rk + Bk.transpose() * pSolGiven[k + 1] +
    //                Bk.transpose() * PSolGiven[k + 1] * bk).transpose() *
    //                    (Rk + Bk.transpose() * PSolGiven[k + 1] *
    //                    Bk).inverse() * (rk + Bk.transpose() * pSolGiven[k +
    //                    1] + Bk.transpose() * PSolGiven[k + 1] * bk);
    KSolGiven[k] = -(Rk + Bk.transpose() * PSolGiven[k + 1] * Bk).inverse() *
                   (Sk + Bk.transpose() * PSolGiven[k + 1] * Ak);
    kSolGiven[k] = -(Rk + Bk.transpose() * PSolGiven[k + 1] * Bk).inverse() *
                   (rk + Bk.transpose() * pSolGiven[k + 1] +
                    Bk.transpose() * PSolGiven[k + 1] * bk);
  }

  // Interface
  ocs2::HpipmInterface::OcpSize ocpSize(N, nx, nu);
  ocs2::HpipmInterface hpipmInterface(ocpSize);

  // Solve!
  std::vector<ocs2::vector_t> xSol;
  std::vector<ocs2::vector_t> uSol;
  hpipmInterface.solve(x0, system, cost, nullptr, xSol, uSol, true);

  // get Riccati info from hpipm interface
  std::vector<ocs2::matrix_t> PSol;
  std::vector<ocs2::matrix_t> KSol;
  std::vector<ocs2::vector_t> pSol;
  std::vector<ocs2::vector_t> kSol;
  hpipmInterface.getRiccatiCostToGo(PSol, pSol);
  hpipmInterface.getRiccatiFeedbackFeedforward(KSol, kSol);
  hpipmInterface.getRiccatiZeroStage(system[0].dfdx, system[0].dfdu,
                                     system[0].f, cost[0].dfdxx, cost[0].dfduu,
                                     cost[0].dfdux, cost[0].dfdx, cost[0].dfdu,
                                     PSol[0], KSol[0], pSol[0], kSol[0]);

  // compare the two vector/matrix trajectory
  ASSERT_TRUE(ocs2::qp_solver::isEqual(PSolGiven, PSol, 1e-9));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(KSolGiven, KSol, 1e-9));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(pSolGiven, pSol, 1e-9));
  ASSERT_TRUE(ocs2::qp_solver::isEqual(kSolGiven, kSol, 1e-9));

  for (int k = 0; k < N; k++) {
    // u* = Kx* + k
    ASSERT_TRUE(uSol[k].isApprox(KSol[k] * xSol[k] + kSol[k]));
  }
}