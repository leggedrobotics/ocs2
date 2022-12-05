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

#include "ocs2_oc/test/SparseScaling.h"

#include <atomic>
#include <numeric>

namespace ocs2 {

// Internal helper functions
namespace {

vector_t matrixInfNormRows(const Eigen::SparseMatrix<scalar_t>& mat) {
  vector_t infNorm;
  infNorm.setZero(mat.rows());

  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      int i = it.row();
      infNorm(i) = std::max(infNorm(i), std::abs(it.value()));
    }
  }
  return infNorm;
}

vector_t matrixInfNormCols(const Eigen::SparseMatrix<scalar_t>& mat) {
  vector_t infNorm;
  infNorm.setZero(mat.cols());

  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      infNorm(j) = std::max(infNorm(j), std::abs(it.value()));
    }
  }
  return infNorm;
}

void scaleMatrixInPlace(const vector_t& rowScale, const vector_t& colScale, Eigen::SparseMatrix<scalar_t>& mat) {
  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      if (it.row() > rowScale.size() - 1 || it.col() > colScale.size() - 1) {
        throw std::runtime_error("[scaleMatrixInPlace] it.row() > rowScale.size() - 1 || it.col() > colScale.size() - 1");
      }
      it.valueRef() *= rowScale(it.row()) * colScale(j);
    }
  }
}
}  // anonymous namespace

void preConditioningSparseMatrixInPlace(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G,
                                        const int iteration, vector_t& DOut, vector_t& EOut, scalar_t& cOut) {
  const int nz = H.rows();
  const int nc = G.rows();

  // Init output
  cOut = 1.0;
  DOut.setOnes(nz);
  EOut.setOnes(nc);

  for (int i = 0; i < iteration; i++) {
    vector_t D, E;
    const vector_t infNormOfHCols = matrixInfNormCols(H);
    const vector_t infNormOfGCols = matrixInfNormCols(G);

    D = infNormOfHCols.array().max(infNormOfGCols.array());
    E = matrixInfNormRows(G);

    for (int i = 0; i < D.size(); i++) {
      if (D(i) > 1e+4) D(i) = 1e+4;
      if (D(i) < 1e-4) D(i) = 1.0;
    }

    for (int i = 0; i < E.size(); i++) {
      if (E(i) > 1e+4) E(i) = 1e+4;
      if (E(i) < 1e-4) E(i) = 1.0;
    }

    D = D.array().sqrt().inverse();
    E = E.array().sqrt().inverse();

    scaleMatrixInPlace(D, D, H);

    scaleMatrixInPlace(E, D, G);

    h.array() *= D.array();

    DOut = DOut.cwiseProduct(D);
    EOut = EOut.cwiseProduct(E);

    scalar_t infNormOfh = h.lpNorm<Eigen::Infinity>();
    if (infNormOfh < 1e-4) infNormOfh = 1.0;

    const vector_t infNormOfHColsUpdated = matrixInfNormCols(H);
    scalar_t c_temp = std::max(infNormOfHColsUpdated.mean(), infNormOfh);
    if (c_temp > 1e+4) c_temp = 1e+4;
    if (c_temp < 1e-4) c_temp = 1.0;

    scalar_t gamma = 1.0 / c_temp;

    H *= gamma;
    h *= gamma;

    cOut *= gamma;
  }
}

void scaleDataInPlace(const OcpSize& ocpSize, const vector_t& D, const vector_t& E, const scalar_t c,
                      std::vector<VectorFunctionLinearApproximation>& dynamics, std::vector<ScalarFunctionQuadraticApproximation>& cost,
                      std::vector<vector_t>& scalingVectors) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }

  scalingVectors.resize(N);

  // Scale H & h
  const int nu_0 = ocpSize.numInputs.front();
  // Scale row - Pre multiply D
  cost.front().dfduu.array().colwise() *= c * D.head(nu_0).array();
  // Scale col - Post multiply D
  cost.front().dfduu *= D.head(nu_0).asDiagonal();

  cost.front().dfdu.array() *= c * D.head(nu_0).array();

  cost.front().dfdux.array().colwise() *= c * D.head(nu_0).array();

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    auto& Q = cost[k].dfdxx;
    auto& R = cost[k].dfduu;
    auto& P = cost[k].dfdux;
    auto& q = cost[k].dfdx;
    auto& r = cost[k].dfdu;

    Q.array().colwise() *= c * D.segment(currRow, nx_k).array();
    Q *= D.segment(currRow, nx_k).asDiagonal();
    q.array() *= c * D.segment(currRow, nx_k).array();

    P *= D.segment(currRow, nx_k).asDiagonal();
    currRow += nx_k;

    R.array().colwise() *= c * D.segment(currRow, nu_k).array();
    R *= D.segment(currRow, nu_k).asDiagonal();
    r.array() *= c * D.segment(currRow, nu_k).array();

    P.array().colwise() *= c * D.segment(currRow, nu_k).array();

    currRow += nu_k;
  }

  const int nx_N = ocpSize.numStates[N];
  cost[N].dfdxx.array().colwise() *= c * D.tail(nx_N).array();
  cost[N].dfdxx *= D.tail(nx_N).asDiagonal();
  cost[N].dfdx.array() *= c * D.tail(nx_N).array();

  // Scale G & g
  auto& B_0 = dynamics[0].dfdu;
  auto& C_0 = scalingVectors[0];
  const int nx_1 = ocpSize.numStates[1];

  //  \tilde{B} = E * B * D,
  //  scaling = E * D,
  //  \tilde{g} = E * g
  B_0.array().colwise() *= E.head(nx_1).array();
  B_0 *= D.head(nu_0).asDiagonal();

  C_0 = E.head(nx_1).array() * D.segment(nu_0, nx_1).array();

  dynamics[0].dfdx.array().colwise() *= E.head(nx_1).array();
  dynamics[0].f.array() *= E.head(nx_1).array();

  currRow = nx_1;
  int currCol = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nu_k = ocpSize.numInputs[k];
    const int nx_k = ocpSize.numStates[k];
    const int nx_next = ocpSize.numStates[k + 1];
    auto& A_k = dynamics[k].dfdx;
    auto& B_k = dynamics[k].dfdu;
    auto& b_k = dynamics[k].f;
    auto& C_k = scalingVectors[k];

    A_k.array().colwise() *= E.segment(currRow, nx_next).array();
    A_k *= D.segment(currCol, nx_k).asDiagonal();

    B_k.array().colwise() *= E.segment(currRow, nx_next).array();
    B_k *= D.segment(currCol + nx_k, nu_k).asDiagonal();

    C_k = E.segment(currRow, nx_next).array() * D.segment(currCol + nx_k + nu_k, nx_next).array();

    b_k.array() *= E.segment(currRow, nx_next).array();

    currRow += nx_next;
    currCol += nx_k + nu_k;
  }
}

}  // namespace ocs2