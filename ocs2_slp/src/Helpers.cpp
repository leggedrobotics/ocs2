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

#include "ocs2_slp/Helpers.h"

#include <atomic>
#include <numeric>

namespace {
int getNumDecisionVariables(const ocs2::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numInputs.begin(), ocpSize.numInputs.end(),
                         std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0));
}

int getNumDynamicsConstraints(const ocs2::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0);
}

int getNumGeneralEqualityConstraints(const ocs2::OcpSize& ocpSize) {
  return std::accumulate(ocpSize.numIneqConstraints.begin(), ocpSize.numIneqConstraints.end(), (int)0);
}
}  // anonymous namespace

namespace ocs2 {
namespace slp {

scalar_t hessianEigenvaluesUpperBound(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost) {
  const vector_t rowwiseAbsSumH = hessianAbsRowSum(ocpSize, cost);
  return rowwiseAbsSumH.maxCoeff();
}

scalar_t GGTEigenvaluesUpperBound(ThreadPool& threadPool, const OcpSize& ocpSize,
                                  const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<VectorFunctionLinearApproximation>* constraintsPtr,
                                  const vector_array_t* scalingVectorsPtr) {
  const vector_t rowwiseAbsSumGGT = GGTAbsRowSumInParallel(threadPool, ocpSize, dynamics, constraintsPtr, scalingVectorsPtr);
  return rowwiseAbsSumGGT.maxCoeff();
}

vector_t hessianAbsRowSum(const OcpSize& ocpSize, const std::vector<ScalarFunctionQuadraticApproximation>& cost) {
  const int N = ocpSize.numStages;
  const int nu_0 = ocpSize.numInputs[0];
  vector_t res(getNumDecisionVariables(ocpSize));
  res.head(nu_0) = cost[0].dfduu.cwiseAbs().rowwise().sum();

  int curRow = nu_0;
  for (int k = 1; k < N; k++) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    if (cost[k].dfdxx.size() != 0) {
      res.segment(curRow, nx_k) = cost[k].dfdxx.cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfdux.size() != 0) {
      res.segment(curRow, nx_k) += cost[k].dfdux.transpose().cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfdux.size() != 0) {
      res.segment(curRow + nx_k, nu_k) = cost[k].dfdux.cwiseAbs().rowwise().sum();
    }
    if (cost[k].dfduu.size() != 0) {
      res.segment(curRow + nx_k, nu_k) += cost[k].dfduu.cwiseAbs().rowwise().sum();
    }
    curRow += nx_k + nu_k;
  }
  const int nx_N = ocpSize.numStates[N];
  res.tail(nx_N) = cost[N].dfdxx.cwiseAbs().rowwise().sum();
  return res;
}

vector_t GGTAbsRowSumInParallel(ThreadPool& threadPool, const OcpSize& ocpSize,
                                const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                const std::vector<VectorFunctionLinearApproximation>* constraints,
                                const vector_array_t* scalingVectorsPtr) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[GGTAbsRowSumInParallel] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[GGTAbsRowSumInParallel] The size of scalingVectors doesn't match the number of stage.");
  }
  Eigen::setNbThreads(1);  // No multithreading within Eigen.

  matrix_array_t tempMatrixArray(N);
  vector_array_t absRowSumArray(N);

  std::atomic_int timeIndex{0};
  auto task = [&](int workerId) {
    int k;
    while ((k = timeIndex++) < N) {
      const auto nx_next = ocpSize.numStates[k + 1];
      const auto& B = dynamics[k].dfdu;
      tempMatrixArray[k] =
          (scalingVectorsPtr == nullptr ? matrix_t::Identity(nx_next, nx_next)
                                        : (*scalingVectorsPtr)[k].cwiseProduct((*scalingVectorsPtr)[k]).asDiagonal().toDenseMatrix());
      tempMatrixArray[k] += B * B.transpose();

      if (k != 0) {
        const auto& A = dynamics[k].dfdx;
        tempMatrixArray[k] += A * A.transpose();
      }

      absRowSumArray[k] = tempMatrixArray[k].cwiseAbs().rowwise().sum();

      if (k != 0) {
        const auto& A = dynamics[k].dfdx;
        absRowSumArray[k] += (A * (scalingVectorsPtr == nullptr ? matrix_t::Identity(A.cols(), A.cols())
                                                                : (*scalingVectorsPtr)[k - 1].asDiagonal().toDenseMatrix()))
                                 .cwiseAbs()
                                 .rowwise()
                                 .sum();
      }
      if (k != N - 1) {
        const auto& ANext = dynamics[k + 1].dfdx;
        absRowSumArray[k] += (ANext * (scalingVectorsPtr == nullptr ? matrix_t::Identity(ANext.cols(), ANext.cols())
                                                                    : (*scalingVectorsPtr)[k].asDiagonal().toDenseMatrix()))
                                 .transpose()
                                 .cwiseAbs()
                                 .rowwise()
                                 .sum();
      }
    }
  };
  threadPool.runParallel(std::move(task), threadPool.numThreads() + 1U);

  vector_t res = vector_t::Zero(getNumDynamicsConstraints(ocpSize));
  int curRow = 0;
  for (const auto& v : absRowSumArray) {
    res.segment(curRow, v.size()) = v;
    curRow += v.size();
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  return res;
}

}  // namespace slp
}  // namespace ocs2
