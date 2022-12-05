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

#include "ocs2_oc/pre_condition/Scaling.h"

#include <atomic>
#include <numeric>

namespace ocs2 {

// Internal helper functions
namespace {

template <typename T>
vector_t matrixInfNormRows(const Eigen::MatrixBase<T>& mat) {
  if (mat.rows() == 0 || mat.cols() == 0) {
    return vector_t(0);
  } else {
    return mat.array().abs().matrix().rowwise().maxCoeff();
  }
}

template <typename T, typename... Rest>
vector_t matrixInfNormRows(const Eigen::MatrixBase<T>& mat, const Eigen::MatrixBase<Rest>&... rest) {
  vector_t temp = matrixInfNormRows(rest...);
  if (mat.rows() == 0 || mat.cols() == 0) {
    return temp;
  } else if (temp.rows() != 0) {
    return mat.array().abs().matrix().rowwise().maxCoeff().cwiseMax(temp);
  } else {
    return mat.array().abs().matrix().rowwise().maxCoeff();
  }
}

template <typename T>
vector_t matrixInfNormCols(const Eigen::MatrixBase<T>& mat) {
  if (mat.rows() == 0 || mat.cols() == 0) {
    return vector_t(0);
  } else {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose();
  }
}

template <typename T, typename... Rest>
vector_t matrixInfNormCols(const Eigen::MatrixBase<T>& mat, const Eigen::MatrixBase<Rest>&... rest) {
  vector_t temp = matrixInfNormCols(rest...);
  if (mat.rows() == 0 || mat.cols() == 0) {
    return temp;
  } else if (temp.rows() != 0) {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose().cwiseMax(temp);
  } else {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose();
  }
}

template <typename T>
void scaleMatrixInPlace(const vector_t* rowScale, const vector_t* colScale, Eigen::MatrixBase<T>& mat) {
  if (rowScale != nullptr) {
    mat.array().colwise() *= rowScale->array();
  }
  if (colScale != nullptr) {
    mat *= colScale->asDiagonal();
  }
}

void invSqrtInfNormInParallel(ThreadPool& threadPool, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                              const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& scalingVectors,
                              const int N, const std::function<void(vector_t&)>& limitScaling, vector_array_t& D, vector_array_t& E) {
  // Helper function
  auto procedure = [&limitScaling](vector_t& dst, const vector_t& norm) {
    dst = norm;
    limitScaling(dst);
    dst.array() = dst.array().sqrt().inverse();
  };

  procedure(D[0], matrixInfNormCols(cost[0].dfduu, dynamics[0].dfdu));
  procedure(E[0], matrixInfNormRows(dynamics[0].dfdu, scalingVectors[0].asDiagonal().toDenseMatrix()));

  std::atomic_int timeStamp{1};
  auto task = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      procedure(D[2 * k - 1],
                matrixInfNormCols(cost[k].dfdxx, cost[k].dfdux, scalingVectors[k - 1].asDiagonal().toDenseMatrix(), dynamics[k].dfdx));
      procedure(D[2 * k], matrixInfNormCols(cost[k].dfdux.transpose().eval(), cost[k].dfduu, dynamics[k].dfdu));
      procedure(E[k], matrixInfNormRows(dynamics[k].dfdx, dynamics[k].dfdu, scalingVectors[k].asDiagonal().toDenseMatrix()));
    }
  };

  threadPool.runParallel(std::move(task), threadPool.numThreads() + 1U);

  procedure(D[2 * N - 1], matrixInfNormCols(cost[N].dfdxx, scalingVectors[N - 1].asDiagonal().toDenseMatrix()));
}

void scaleDataOneStepInPlaceInParallel(ThreadPool& threadPool, const vector_array_t& D, const vector_array_t& E, const int N,
                                       std::vector<VectorFunctionLinearApproximation>& dynamics,
                                       std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors) {
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdu);
  scaleMatrixInPlace(&(D[0]), &(D[0]), cost.front().dfduu);
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdux);

  std::atomic_int timeStamp{1};
  auto scaleCost = [&](int workerId) {
    int k;
    while ((k = timeStamp++) <= N) {
      scaleMatrixInPlace(&(D[2 * k - 1]), nullptr, cost[k].dfdx);
      scaleMatrixInPlace(&(D[2 * k - 1]), &(D[2 * k - 1]), cost[k].dfdxx);
      if (cost[k].dfdu.size() > 0) {
        scaleMatrixInPlace(&(D[2 * k]), nullptr, cost[k].dfdu);
        scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k]), cost[k].dfduu);
        scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k - 1]), cost[k].dfdux);
      }
    }
  };
  threadPool.runParallel(std::move(scaleCost), threadPool.numThreads() + 1U);

  // Scale G & g
  /**
   * \f$
   * \tilde{B} = E * B * D,
   * scaling = E * I * D,
   * \tilde{g} = E * g,
   * \f$
   */
  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].f);
  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].dfdx);
  scalingVectors[0].array() *= E[0].array() * D[1].array();
  if (dynamics[0].dfdu.size() > 0) {
    scaleMatrixInPlace(&(E[0]), &(D[0]), dynamics[0].dfdu);
  }

  timeStamp = 1;
  auto scaleConstraints = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      scaleMatrixInPlace(&(E[k]), nullptr, dynamics[k].f);
      scaleMatrixInPlace(&(E[k]), &(D[2 * k - 1]), dynamics[k].dfdx);
      scalingVectors[k].array() *= E[k].array() * D[2 * k + 1].array();
      if (dynamics[k].dfdu.size() > 0) {
        scaleMatrixInPlace(&(E[k]), &(D[2 * k]), dynamics[k].dfdu);
      }
    }
  };
  threadPool.runParallel(std::move(scaleConstraints), threadPool.numThreads() + 1U);
}
}  // anonymous namespace

void preConditioningInPlaceInParallel(ThreadPool& threadPool, const vector_t& x0, const OcpSize& ocpSize, const int iteration,
                                      std::vector<VectorFunctionLinearApproximation>& dynamics,
                                      std::vector<ScalarFunctionQuadraticApproximation>& cost, vector_array_t& DOut, vector_array_t& EOut,
                                      vector_array_t& scalingVectors, scalar_t& cOut) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[preConditioningInPlaceInParallel] The number of stages cannot be less than 1.");
  }

  auto limitScaling = [](vector_t& vec) {
    for (int i = 0; i < vec.size(); i++) {
      if (vec(i) > 1e+4) vec(i) = 1e+4;
      if (vec(i) < 1e-4) vec(i) = 1.0;
    }
  };

  // Init output
  cOut = 1.0;
  DOut.resize(2 * N);
  EOut.resize(N);
  scalingVectors.resize(N);
  for (int i = 0; i < N; i++) {
    DOut[2 * i].setOnes(ocpSize.numInputs[i]);
    DOut[2 * i + 1].setOnes(ocpSize.numStates[i + 1]);
    EOut[i].setOnes(ocpSize.numStates[i + 1]);
    scalingVectors[i].setOnes(ocpSize.numStates[i + 1]);
  }

  vector_array_t D(2 * N), E(N);
  for (int i = 0; i < iteration; i++) {
    invSqrtInfNormInParallel(threadPool, dynamics, cost, scalingVectors, N, limitScaling, D, E);
    scaleDataOneStepInPlaceInParallel(threadPool, D, E, N, dynamics, cost, scalingVectors);

    for (int k = 0; k < DOut.size(); k++) {
      DOut[k].array() *= D[k].array();
    }
    for (int k = 0; k < EOut.size(); k++) {
      EOut[k].array() *= E[k].array();
    }

    scalar_t infNormOfh = (cost.front().dfdu + cost.front().dfdux * x0).lpNorm<Eigen::Infinity>();
    for (int k = 1; k < N; k++) {
      infNormOfh = std::max(infNormOfh, std::max(cost[k].dfdx.lpNorm<Eigen::Infinity>(), cost[k].dfdu.lpNorm<Eigen::Infinity>()));
    }
    infNormOfh = std::max(infNormOfh, cost[N].dfdx.lpNorm<Eigen::Infinity>());
    if (infNormOfh < 1e-4) infNormOfh = 1.0;

    scalar_t sumOfInfNormOfH = matrixInfNormCols(cost[0].dfduu).derived().sum();
    for (int k = 1; k < N; k++) {
      sumOfInfNormOfH += matrixInfNormCols(cost[k].dfdxx, cost[k].dfdux).derived().sum();
      sumOfInfNormOfH += matrixInfNormCols(cost[k].dfdux.transpose().eval(), cost[k].dfduu).derived().sum();
    }
    sumOfInfNormOfH += matrixInfNormCols(cost[N].dfdxx).derived().sum();

    const int numDecisionVariables = std::accumulate(ocpSize.numInputs.begin(), ocpSize.numInputs.end(), (int)0) +
                                     std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0);

    scalar_t c_temp = std::max(sumOfInfNormOfH / static_cast<scalar_t>(numDecisionVariables), infNormOfh);
    if (c_temp > 1e+4) c_temp = 1e+4;
    if (c_temp < 1e-4) c_temp = 1.0;

    const scalar_t gamma = 1.0 / c_temp;

    for (int k = 0; k <= N; ++k) {
      cost[k].dfdxx *= gamma;
      cost[k].dfduu *= gamma;
      cost[k].dfdux *= gamma;
      cost[k].dfdx *= gamma;
      cost[k].dfdu *= gamma;
    }

    cOut *= gamma;
  }
}

}  // namespace ocs2