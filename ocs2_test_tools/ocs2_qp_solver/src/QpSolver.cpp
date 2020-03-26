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

#include "ocs2_qp_solver/QpSolver.h"

#include <Eigen/LU>
#include <numeric>

namespace ocs2 {
namespace qp_solver {

/**
 * Extracts the problem state and inputs dimensions from a linear quadratic approximation
 * Looks at the size of the flowmap derivatives of the dynamics.
 * @return { numStatesPerStage, numInputsPerStage }
 */
std::pair<std::vector<int>, std::vector<int>> getNumStatesAndInputs(const std::vector<LinearQuadraticStage>& linearQuadraticApproximation) {
  const int N = linearQuadraticApproximation.size() - 1;
  std::vector<int> numStates;
  std::vector<int> numInputs;
  numStates.reserve(N + 1);
  numInputs.reserve(N);

  for (int k = 0; k < N; ++k) {
    numStates.push_back(linearQuadraticApproximation[k].dynamics.dfdx.cols());
    numInputs.push_back(linearQuadraticApproximation[k].dynamics.dfdu.cols());
  }
  numStates.push_back(linearQuadraticApproximation[N - 1].dynamics.dfdx.rows());

  return {numStates, numInputs};
}

/** Counts the number of decision variables in the QP */
int getNumDecisionVariables(const std::vector<int>& numStates, const std::vector<int>& numInputs) {
  int totalNumberOfStates = std::accumulate(numStates.begin(), numStates.end(), 0);
  int totalNumberOfInputs = std::accumulate(numInputs.begin(), numInputs.end(), 0);
  return totalNumberOfStates + totalNumberOfInputs;
}

/** Counts the number of constraints in the QP */
int getNumConstraints(const std::vector<int>& numStates) {
  // Each stage constrains x_{k+1} states, adding the x_0 constraint, all states are constrained exactly once.
  return std::accumulate(numStates.begin(), numStates.end(), 0);
}

ContinuousTrajectory solveLinearQuadraticApproximation(const std::vector<LinearQuadraticStage>& lqApproximation,
                                                       const ContinuousTrajectory& nominalTrajectory,
                                                       const dynamic_vector_t& initialState) {
  // Extract sizes
  std::vector<int> numStates;
  std::vector<int> numInputs;
  std::tie(numStates, numInputs) = getNumStatesAndInputs(lqApproximation);
  const int numDecisionVariables = getNumDecisionVariables(numStates, numInputs);
  const int numConstraints = getNumConstraints(numStates);

  // Construct QP
  const auto constraints = getConstraintMatrices(lqApproximation, initialState - nominalTrajectory.stateTrajectory.front(), numConstraints,
                                                 numDecisionVariables);
  const auto costs = getCostMatrices(lqApproximation, numDecisionVariables);

  // Solve
  const auto primalDualSolution = solveDenseQp(costs, constraints);

  // Extract solution
  ContinuousTrajectory deltaSolution;
  deltaSolution.timeTrajectory = nominalTrajectory.timeTrajectory;
  std::tie(deltaSolution.stateTrajectory, deltaSolution.inputTrajectory) =
      getStateAndInputTrajectory(numStates, numInputs, primalDualSolution.first);
  return deltaSolution;
}

VectorFunctionLinearApproximation getConstraintMatrices(const std::vector<LinearQuadraticStage>& lqp, const dynamic_vector_t& dx0,
                                                        int numConstraints, int numDecisionVariables) {
  const int N = lqp.size() - 1;

  // Preallocate full constraint matrix
  VectorFunctionLinearApproximation constraints;
  auto& A = constraints.dfdx;
  auto& b = constraints.f;
  A.setZero(numConstraints, numDecisionVariables);
  b.setZero(numConstraints);

  // Initial state constraint
  const int nx_0 = dx0.size();
  A.topLeftCorner(nx_0, nx_0).setIdentity();
  b.topRows(nx_0) << -dx0;

  int currRow = nx_0;
  int currCol = 0;
  for (int k = 0; k < N; ++k) {
    const auto& dynamics_k = lqp[k].dynamics;
    const int nu_k = dynamics_k.dfdu.cols();
    const int nx_k = dynamics_k.dfdx.cols();
    const int nx_Next = dynamics_k.dfdx.rows();

    // Add [A, B, -I]
    A.block(currRow, currCol, nx_Next, nx_k + nu_k + nx_Next) << dynamics_k.dfdx, dynamics_k.dfdu,
        -dynamic_matrix_t::Identity(nx_Next, nx_Next);
    // Add [b]
    b.segment(currRow, nx_Next) << dynamics_k.f;

    currRow += nx_Next;
    currCol += nx_k + nu_k;
  }

  return constraints;
}

ScalarFunctionQuadraticApproximation getCostMatrices(const std::vector<LinearQuadraticStage>& lqp, int numDecisionVariables) {
  const int N = lqp.size() - 1;

  // Preallocate full Cost matrices
  ScalarFunctionQuadraticApproximation qpCost;
  auto& H = qpCost.dfdxx;
  auto& g = qpCost.dfdx;
  auto& c = qpCost.f;
  H.setZero(numDecisionVariables, numDecisionVariables);
  g.setZero(numDecisionVariables);
  c = 0.0;

  int currRow = 0;
  for (int k = 0; k < N; ++k) {
    const auto& cost_k = lqp[k].cost;
    const int nx_k = cost_k.dfdux.cols();
    const int nu_k = cost_k.dfdux.rows();

    // Add [ Q, P'
    //       P, R ]
    H.block(currRow, currRow, nx_k + nu_k, nx_k + nu_k) << cost_k.dfdxx, cost_k.dfdux.transpose(), cost_k.dfdux, cost_k.dfduu;
    // Add [ q, r]
    g.segment(currRow, nx_k + nu_k) << cost_k.dfdx, cost_k.dfdu;
    // Add nominal cost
    c += cost_k.f;

    currRow += nx_k + nu_k;
  }

  // Terminal cost
  const auto& cost_N = lqp[N].cost;
  const int nx_N = cost_N.dfdx.size();
  H.block(currRow, currRow, nx_N, nx_N) << cost_N.dfdxx;
  g.segment(currRow, nx_N) << cost_N.dfdx;
  c += cost_N.f;

  return qpCost;
}

std::pair<dynamic_vector_t, dynamic_vector_t> solveDenseQp(const ScalarFunctionQuadraticApproximation& cost,
                                                           const VectorFunctionLinearApproximation& constraints) {
  const int m = constraints.dfdx.rows();
  const int n = constraints.dfdx.cols();

  // Assemble KKT condition
  dynamic_matrix_t kktMatrix(n + m, n + m);
  dynamic_vector_t kktRhs(n + m);
  kktMatrix << cost.dfdxx, constraints.dfdx.transpose(), constraints.dfdx, dynamic_matrix_t::Zero(m, m);
  kktRhs << -cost.dfdx, -constraints.f;

  assert(kktMatrix.fullPivLu().rank() == n + m);  // prerequisite for the LU factorization, and the solution would be non-unique.
  dynamic_vector_t sol = kktMatrix.lu().solve(kktRhs);
  return {sol.head(n), sol.tail(m)};
}

std::pair<dynamic_vector_array_t, dynamic_vector_array_t> getStateAndInputTrajectory(const std::vector<int>& numStates,
                                                                                     const std::vector<int>& numInputs,
                                                                                     const dynamic_vector_t& w) {
  const int N = numInputs.size();

  dynamic_vector_array_t stateTrajectory;
  dynamic_vector_array_t inputTrajectory;
  stateTrajectory.reserve(N + 1);
  inputTrajectory.reserve(N);

  int index = 0;
  for (int k = 0; k < N; ++k) {
    // x[k]
    stateTrajectory.emplace_back(w.segment(index, numStates[k]));
    index += numStates[k];

    // u[k]
    inputTrajectory.emplace_back(w.segment(index, numInputs[k]));
    index += numInputs[k];
  }

  // x[N]
  stateTrajectory.emplace_back(w.segment(index, numStates[N]));

  return {stateTrajectory, inputTrajectory};
}

}  // namespace qp_solver
}  // namespace ocs2
