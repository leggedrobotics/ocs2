//
// Created by rgrandia on 26.02.20.
//

#include "ocs2_qp_solver/QpSolver.h"

#include <Eigen/Cholesky>
#include <numeric>

namespace ocs2_qp_solver {

ContinuousTrajectory solveLinearQuadraticApproximation(const std::vector<LinearQuadraticStage>& lqApproximation,
                                                       const ProblemDimensions& problemDimensions,
                                                       const ContinuousTrajectory& linearizationTrajectory,
                                                       const Eigen::VectorXd& initialState) {
  // Construct QP
  const auto constraints =
      getConstraintMatrices(problemDimensions, lqApproximation, initialState - linearizationTrajectory.stateTrajectory.front());
  const auto costs = getCostMatrices(problemDimensions, lqApproximation);

  // Solve
  const auto primalDualSolution = solveDenseQp(costs, constraints);

  // Extract solution
  ContinuousTrajectory relativeSolution;
  relativeSolution.timeTrajectory = linearizationTrajectory.timeTrajectory;
  std::tie(relativeSolution.stateTrajectory, relativeSolution.inputTrajectory) =
      getStateAndInputTrajectory(problemDimensions, primalDualSolution.first);
  return relativeSolution;
}

int getNumDecisionVariables(const ProblemDimensions& problemDimensions) {
  int totalNumberOfStates = std::accumulate(problemDimensions.numStates.begin(), problemDimensions.numStates.end(), 0);
  int totalNumberOfInputs = std::accumulate(problemDimensions.numInputs.begin(), problemDimensions.numInputs.end(), 0);
  return totalNumberOfStates + totalNumberOfInputs;
}

int getNumConstraints(const ProblemDimensions& problemDimensions) {
  // Each stage constrains x_{k+1} states, adding the x_0 constraint, all states are constrained exactly once.
  return std::accumulate(problemDimensions.numStates.begin(), problemDimensions.numStates.end(), 0);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> getConstraintMatrices(const ProblemDimensions& dims,
                                                                  const std::vector<LinearQuadraticStage>& lqp,
                                                                  const Eigen::VectorXd& dx0) {
  const auto N = dims.numStages;

  // Preallocate full constraint matrix
  const int m = getNumConstraints(dims);
  const int n = getNumDecisionVariables(dims);
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m, n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(m);

  // Initial state constraint
  const int nx_0 = dims.numStates[0];
  A.topLeftCorner(nx_0, nx_0).setIdentity();
  b.topRows(nx_0) << dx0;

  int currRow = nx_0;
  int currCol = 0;
  for (int k = 0; k < N; ++k) {
    const auto nu_k = dims.numInputs[k];
    const auto nx_k = dims.numStates[k];
    const auto nx_Next = dims.numStates[k + 1];
    const auto& dynamics_k = lqp[k].dynamics;

    // Add [A, B, -I]
    A.block(currRow, currCol, nx_Next, nx_k + nu_k + nx_Next) << dynamics_k.A, dynamics_k.B, -Eigen::MatrixXd::Identity(nx_Next, nx_Next);
    // Add [-b]
    b.segment(currRow, nx_Next) << -dynamics_k.b;

    currRow += nx_Next;
    currCol += nx_k + nu_k;
  }

  return {A, b};
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> getCostMatrices(const ProblemDimensions& dims, const std::vector<LinearQuadraticStage>& lqp) {
  const auto N = dims.numStages;

  // Preallocate full Cost matrix
  const int n = getNumDecisionVariables(dims);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n, n);
  Eigen::VectorXd g(n);

  int currRow = 0;
  for (int k = 0; k < N; ++k) {
    const auto nx_k = dims.numStates[k];
    const auto nu_k = dims.numInputs[k];
    const auto& cost_k = lqp[k].cost;

    // Add [ Q, P'
    //       P, R ]
    H.block(currRow, currRow, nx_k + nu_k, nx_k + nu_k) << cost_k.Q, cost_k.P.transpose(), cost_k.P, cost_k.R;
    // Add [q, r]
    g.segment(currRow, nx_k + nu_k) << cost_k.q, cost_k.r;

    currRow += nx_k + nu_k;
  }

  // Terminal cost
  const auto nx_N = dims.numStates[N];
  H.block(currRow, currRow, nx_N, nx_N) << lqp[N].cost.Q;
  g.segment(currRow, nx_N) << lqp[N].cost.q;

  return {H, g};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> solveDenseQp(const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& Hg,
                                                         const std::pair<Eigen::MatrixXd, Eigen::VectorXd>& Ab) {
  const auto& H = Hg.first;
  const auto& g = Hg.second;
  const auto& A = Ab.first;
  const auto& b = Ab.second;
  const int m = A.rows();
  const int n = H.cols();

  // Assemble KKT condition
  Eigen::MatrixXd kktMatrix(n + m, n + m);
  Eigen::VectorXd kktRhs(n + m);
  kktMatrix << H, A.transpose(), A, Eigen::MatrixXd::Zero(m, m);
  kktRhs << -g, b;

  assert(kktMatrix.fullPivLu().rank() == n + m);  // prerequisite for the LU factorization, and the solution would be non-unique.
  Eigen::VectorXd sol = kktMatrix.lu().solve(kktRhs);
  return {sol.head(n), sol.tail(m)};
}

std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::VectorXd>> getStateAndInputTrajectory(const ProblemDimensions& dims,
                                                                                                 const Eigen::VectorXd& w) {
  const auto N = dims.numStages;

  std::vector<Eigen::VectorXd> stateTrajectory;
  std::vector<Eigen::VectorXd> inputTrajectory;
  stateTrajectory.reserve(N + 1);
  inputTrajectory.reserve(N);

  int index = 0;
  for (int k = 0; k < N; ++k) {
    // x[k]
    stateTrajectory.emplace_back(w.segment(index, dims.numStates[k]));
    index += dims.numStates[k];

    // u[k]
    inputTrajectory.emplace_back(w.segment(index, dims.numInputs[k]));
    index += dims.numInputs[k];
  }

  // x[N]
  stateTrajectory.emplace_back(w.segment(index, dims.numStates[N]));

  return {stateTrajectory, inputTrajectory};
}

}  // namespace ocs2_qp_solver
