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

#include "ocs2_oc/oc_problem/OcpToKkt.h"

#include <numeric>

namespace ocs2 {
// Helper functions
namespace {
int getNumDecisionVariables(const OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numInputs.begin(), ocpSize.numInputs.end(),
                         std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0));
}

int getNumDynamicsConstraints(const OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0);
}

int getNumGeneralEqualityConstraints(const OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numIneqConstraints.begin(), ocpSize.numIneqConstraints.end(), (int)0);
}
}  // namespace

void getConstraintMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                         const std::vector<VectorFunctionLinearApproximation>* constraintsPtr, const vector_array_t* scalingVectorsPtr,
                         VectorFunctionLinearApproximation& res) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[getConstraintMatrix] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[getConstraintMatrix] The size of scalingVectors doesn't match the number of stage.");
  }

  // Preallocate full constraint matrix
  auto& g = res.f;
  auto& G = res.dfdx;
  if (constraintsPtr == nullptr) {
    g.setZero(getNumDynamicsConstraints(ocpSize));
  } else {
    g.setZero(getNumDynamicsConstraints(ocpSize) + getNumGeneralEqualityConstraints(ocpSize));
  }
  G.setZero(g.rows(), getNumDecisionVariables(ocpSize));

  // Initial state constraint
  const int nu_0 = ocpSize.numInputs.front();
  const int nx_1 = ocpSize.numStates[1];
  if (scalingVectorsPtr == nullptr) {
    G.topLeftCorner(nx_1, nu_0 + nx_1) << -dynamics.front().dfdu, matrix_t::Identity(nx_1, nx_1);
  } else {
    G.topLeftCorner(nx_1, nu_0 + nx_1) << -dynamics.front().dfdu, scalingVectorsPtr->front().asDiagonal().toDenseMatrix();
  }

  // k = 0. Absorb initial state into dynamics
  // The initial state is removed from the decision variables
  // The first dynamics becomes:
  //    x[1] = A[0]*x[0] + B[0]*u[0] + b[0]
  //         = B[0]*u[0] + (b[0] + A[0]*x[0])
  //         = B[0]*u[0] + \tilde{b}[0]
  // numState[0] = 0 --> No need to specify A[0] here
  g.head(nx_1) = dynamics.front().f;
  g.head(nx_1).noalias() += dynamics.front().dfdx * x0;

  int currRow = nx_1;
  int currCol = nu_0;
  for (int k = 1; k < N; ++k) {
    const auto& dynamics_k = dynamics[k];
    // const auto& constraints_k = constraints;
    const int nu_k = ocpSize.numInputs[k];
    const int nx_k = ocpSize.numStates[k];
    const int nx_next = ocpSize.numStates[k + 1];

    // Add [-A, -B, I(C)]
    if (scalingVectorsPtr == nullptr) {
      G.block(currRow, currCol, nx_next, nx_k + nu_k + nx_next) << -dynamics_k.dfdx, -dynamics_k.dfdu, matrix_t::Identity(nx_next, nx_next);
    } else {
      G.block(currRow, currCol, nx_next, nx_k + nu_k + nx_next) << -dynamics_k.dfdx, -dynamics_k.dfdu,
          (*scalingVectorsPtr)[k].asDiagonal().toDenseMatrix();
    }

    // Add [b]
    g.segment(currRow, nx_next) = dynamics_k.f;

    currRow += nx_next;
    currCol += nx_k + nu_k;
  }  // end of i loop

  if (constraintsPtr != nullptr) {
    currCol = nu_0;
    // === Constraints ===
    // for ocs2 --> C*dx + D*du + e = 0
    // for pipg --> C*dx + D*du = -e
    // Initial general constraints
    const int nc_0 = ocpSize.numIneqConstraints.front();
    if (nc_0 > 0) {
      const auto& constraint_0 = (*constraintsPtr).front();
      G.block(currRow, 0, nc_0, nu_0) = constraint_0.dfdu;
      g.segment(currRow, nc_0) = -constraint_0.f;
      g.segment(currRow, nc_0) -= constraint_0.dfdx * x0;
      currRow += nc_0;
    }

    for (int k = 1; k < N; ++k) {
      const int nc_k = ocpSize.numIneqConstraints[k];
      const int nu_k = ocpSize.numInputs[k];
      const int nx_k = ocpSize.numStates[k];
      if (nc_k > 0) {
        const auto& constraints_k = (*constraintsPtr)[k];

        // Add [C, D, 0]
        G.block(currRow, currCol, nc_k, nx_k + nu_k) << constraints_k.dfdx, constraints_k.dfdu;
        // Add [-e]
        g.segment(currRow, nc_k) = -constraints_k.f;
        currRow += nc_k;
      }

      currCol += nx_k + nu_k;
    }

    // Final general constraint
    const int nc_N = ocpSize.numIneqConstraints[N];
    if (nc_N > 0) {
      const auto& constraints_N = (*constraintsPtr)[N];
      G.bottomRightCorner(nc_N, constraints_N.dfdx.cols()) = constraints_N.dfdx;
      g.tail(nc_N) = -constraints_N.f;
    }
  }  // end of i loop
}

void getConstraintMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                               const vector_array_t* scalingVectorsPtr, Eigen::SparseMatrix<scalar_t>& G, vector_t& g) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[getConstraintMatrixSparse] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[getConstraintMatrixSparse] The size of scalingVectors doesn't match the number of stage.");
  }

  const int nu_0 = ocpSize.numInputs[0];
  const int nx_1 = ocpSize.numStates[1];
  const int nx_N = ocpSize.numStates[N];

  int nnz = nx_1 * (nu_0 + nx_1);
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    const int nx_next = ocpSize.numStates[k + 1];
    nnz += nx_next * (nx_k + nu_k + nx_next);
  }

  if (constraintsPtr != nullptr) {
    const int nc_0 = ocpSize.numIneqConstraints[0];
    nnz += nc_0 * nu_0;
    for (int k = 1; k < N; ++k) {
      const int nx_k = ocpSize.numStates[k];
      const int nu_k = ocpSize.numInputs[k];
      const int nc_k = ocpSize.numIneqConstraints[k];
      nnz += nc_k * (nx_k + nu_k);
    }
    const int nc_N = ocpSize.numIneqConstraints[N];

    nnz += nc_N * nx_N;
  }
  std::vector<Eigen::Triplet<scalar_t>> tripletList;
  tripletList.reserve(nnz);

  auto emplaceBackMatrix = [&tripletList](const int startRow, const int startCol, const matrix_t& mat) {
    for (int j = 0; j < mat.cols(); j++) {
      for (int i = 0; i < mat.rows(); i++) {
        if (mat(i, j) != 0) {
          tripletList.emplace_back(startRow + i, startCol + j, mat(i, j));
        }
      }
    }
  };

  if (constraintsPtr == nullptr) {
    g.setZero(getNumDynamicsConstraints(ocpSize));
  } else {
    g.setZero(getNumDynamicsConstraints(ocpSize) + getNumGeneralEqualityConstraints(ocpSize));
  }
  G.resize(g.rows(), getNumDecisionVariables(ocpSize));

  // Initial state constraint
  emplaceBackMatrix(0, 0, -dynamics.front().dfdu);
  if (scalingVectorsPtr == nullptr) {
    emplaceBackMatrix(0, nu_0, matrix_t::Identity(nx_1, nx_1));
  } else {
    emplaceBackMatrix(0, nu_0, scalingVectorsPtr->front().asDiagonal());
  }

  // k = 0. Absorb initial state into dynamics
  // The initial state is removed from the decision variables
  // The first dynamics becomes:
  //    x[1] = A[0]*x[0] + B[0]*u[0] + b[0]
  //         = B[0]*u[0] + (b[0] + A[0]*x[0])
  //         = B[0]*u[0] + \tilde{b}[0]
  // numState[0] = 0 --> No need to specify A[0] here
  g.head(nx_1) = dynamics.front().f;
  g.head(nx_1).noalias() += dynamics.front().dfdx * x0;

  int currRow = nx_1;
  int currCol = nu_0;
  for (int k = 1; k < N; ++k) {
    const auto& dynamics_k = dynamics[k];
    // const auto& constraints_k = constraints;
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    const int nx_next = ocpSize.numStates[k + 1];

    // Add [-A, -B, I]
    emplaceBackMatrix(currRow, currCol, -dynamics_k.dfdx);
    emplaceBackMatrix(currRow, currCol + nx_k, -dynamics_k.dfdu);
    if (scalingVectorsPtr == nullptr) {
      emplaceBackMatrix(currRow, currCol + nx_k + nu_k, matrix_t::Identity(nx_next, nx_next));
    } else {
      emplaceBackMatrix(currRow, currCol + nx_k + nu_k, (*scalingVectorsPtr)[k].asDiagonal());
    }

    // Add [b]
    g.segment(currRow, nx_next) = dynamics_k.f;

    currRow += nx_next;
    currCol += nx_k + nu_k;
  }

  if (constraintsPtr != nullptr) {
    currCol = nu_0;
    // === Constraints ===
    // for ocs2 --> C*dx + D*du + e = 0
    // for pipg --> C*dx + D*du = -e
    // Initial general constraints
    const int nc_0 = ocpSize.numIneqConstraints.front();
    if (nc_0 > 0) {
      const auto& constraint_0 = (*constraintsPtr).front();
      emplaceBackMatrix(currRow, 0, constraint_0.dfdu);

      g.segment(currRow, nc_0) = -constraint_0.f;
      g.segment(currRow, nc_0) -= constraint_0.dfdx * x0;
      currRow += nc_0;
    }

    for (int k = 1; k < N; ++k) {
      const int nc_k = ocpSize.numIneqConstraints[k];
      const int nu_k = ocpSize.numInputs[k];
      const int nx_k = ocpSize.numStates[k];
      if (nc_k > 0) {
        const auto& constraints_k = (*constraintsPtr)[k];

        // Add [C, D, 0]
        emplaceBackMatrix(currRow, currCol, constraints_k.dfdx);
        emplaceBackMatrix(currRow, currCol + nx_k, constraints_k.dfdu);
        // Add [-e]
        g.segment(currRow, nc_k) = -constraints_k.f;
        currRow += nc_k;
      }

      currCol += nx_k + nu_k;
    }

    // Final general constraint
    const int nc_N = ocpSize.numIneqConstraints[N];
    if (nc_N > 0) {
      const auto& constraints_N = (*constraintsPtr)[N];
      emplaceBackMatrix(currRow, currCol, constraints_N.dfdx);
      g.segment(currRow, nc_N) = -constraints_N.f;
    }
  }

  G.setFromTriplets(tripletList.begin(), tripletList.end());
  assert(G.nonZeros() <= nnz);
}

void getCostMatrix(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   ScalarFunctionQuadraticApproximation& res) {
  const int N = ocpSize.numStages;

  // Preallocate full Cost matrices
  auto& H = res.dfdxx;
  auto& h = res.dfdx;
  auto& c = res.f;
  H.setZero(getNumDecisionVariables(ocpSize), getNumDecisionVariables(ocpSize));
  h.setZero(H.cols());
  c = 0.0;

  // k = 0. Elimination of initial state requires cost adaptation
  vector_t r_0 = cost[0].dfdu;
  r_0 += cost[0].dfdux * x0;

  const int nu_0 = ocpSize.numInputs[0];
  H.topLeftCorner(nu_0, nu_0) = cost[0].dfduu;
  h.head(nu_0) = r_0;

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];

    // Add [ Q, P'
    //       P, Q ]
    H.block(currRow, currRow, nx_k + nu_k, nx_k + nu_k) << cost[k].dfdxx, cost[k].dfdux.transpose(), cost[k].dfdux, cost[k].dfduu;

    // Add [ q, r]
    h.segment(currRow, nx_k + nu_k) << cost[k].dfdx, cost[k].dfdu;

    // Add nominal cost
    c += cost[k].f;

    currRow += nx_k + nu_k;
  }

  const int nx_N = ocpSize.numStates[N];
  H.bottomRightCorner(nx_N, nx_N) = cost[N].dfdxx;
  h.tail(nx_N) = cost[N].dfdx;
  c += cost[N].f;
}

void getCostMatrixSparse(const OcpSize& ocpSize, const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         Eigen::SparseMatrix<scalar_t>& H, vector_t& h) {
  const int N = ocpSize.numStages;

  const int nu_0 = ocpSize.numInputs[0];
  int nnz = nu_0 * nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    nnz += (nx_k + nu_k) * (nx_k + nu_k);
  }
  const int nx_N = ocpSize.numStates[N];

  nnz += nx_N * nx_N;
  std::vector<Eigen::Triplet<scalar_t>> tripletList;
  tripletList.reserve(nnz);

  auto emplaceBackMatrix = [&tripletList](const int startRow, const int startCol, const matrix_t& mat) {
    for (int j = 0; j < mat.cols(); j++) {
      for (int i = 0; i < mat.rows(); i++) {
        if (mat(i, j) != 0) {
          tripletList.emplace_back(startRow + i, startCol + j, mat(i, j));
        }
      }
    }
  };

  h.setZero(getNumDecisionVariables(ocpSize));

  // k = 0. Elimination of initial state requires cost adaptation
  vector_t r_0 = cost[0].dfdu;
  r_0 += cost[0].dfdux * x0;

  // R0
  emplaceBackMatrix(0, 0, cost[0].dfduu);
  h.head(nu_0) = r_0;

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];

    // Add [ Q, P'
    //       P, Q ]
    emplaceBackMatrix(currRow, currRow, cost[k].dfdxx);
    emplaceBackMatrix(currRow, currRow + nx_k, cost[k].dfdux.transpose());
    emplaceBackMatrix(currRow + nx_k, currRow, cost[k].dfdux);
    emplaceBackMatrix(currRow + nx_k, currRow + nx_k, cost[k].dfduu);

    // Add [ q, r]
    h.segment(currRow, nx_k + nu_k) << cost[k].dfdx, cost[k].dfdu;

    currRow += nx_k + nu_k;
  }

  emplaceBackMatrix(currRow, currRow, cost[N].dfdxx);
  h.tail(nx_N) = cost[N].dfdx;

  H.resize(getNumDecisionVariables(ocpSize), getNumDecisionVariables(ocpSize));
  H.setFromTriplets(tripletList.begin(), tripletList.end());
  assert(H.nonZeros() <= nnz);
}

void toOcpSolution(const OcpSize& ocpSize, const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
                   vector_array_t& uTrajectory) {
  const int N = ocpSize.numStages;
  xTrajectory.resize(N + 1);
  uTrajectory.resize(N);

  xTrajectory.front() = x0;

  int curRow = 0;
  for (int i = 0; i < N; i++) {
    const auto nx = ocpSize.numStates[i + 1];
    const auto nu = ocpSize.numInputs[i];
    xTrajectory[i + 1] = stackedSolution.segment(curRow + nu, nx);
    uTrajectory[i] = stackedSolution.segment(curRow, nu);

    curRow += nx + nu;
  }
}

void toKktSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution) {
  int numDecisionVariables = 0;
  for (int i = 1; i < xTrajectory.size(); i++) {
    numDecisionVariables += uTrajectory[i - 1].size() + xTrajectory[i].size();
  }

  stackedSolution.resize(numDecisionVariables);

  int curRow = 0;
  for (int i = 1; i < xTrajectory.size(); i++) {
    const auto nu = uTrajectory[i - 1].size();
    const auto nx = xTrajectory[i].size();
    stackedSolution.segment(curRow, nx + nu) << uTrajectory[i - 1], xTrajectory[i];
    curRow += nx + nu;
  }
}

}  // namespace ocs2