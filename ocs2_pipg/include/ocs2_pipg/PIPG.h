#pragma once

#include "ocs2_pipg/OcpSize.h"
#include "ocs2_pipg/PIPG_Settings.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ThreadPool.h>

#include <Eigen/Sparse>

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <numeric>

namespace ocs2 {

class Pipg {
 public:
  using Settings = pipg::Settings;
  using OcpSize = pipg::OcpSize;
  // solver status
  enum class SolverStatus { SUCCESS, MAX_ITER };
  static std::string status2string(SolverStatus s) {
    std::string res;
    switch (s) {
      case SolverStatus::SUCCESS:
        res = "SUCCESS";
        break;

      case SolverStatus::MAX_ITER:
        res = "FAIL";
        break;

      default:
        break;
    }
    return res;
  }

  Pipg() = default;
  explicit Pipg(pipg::Settings pipgSettings);
  ~Pipg();

  /**
   * Solve generic QP problem
   *
   * @note For constraints, it is Gz = g
   *
   * @param cost: Hessian(H) and gradient(h).
   * @param constraints: Linear equality constraint matrix(G) and value(g).
   * @param mu
   * @param lambda
   * @param sigma
   * @param result
   * @return SolverStatus
   */
  SolverStatus solveDenseQP(const ScalarFunctionQuadraticApproximation& cost, const VectorFunctionLinearApproximation& constraints,
                            const vector_t& EInv, const scalar_t mu, const scalar_t lambda, const scalar_t sigma, vector_t& result) {
    Eigen::SparseMatrix<scalar_t> H = cost.dfdxx.sparseView();
    auto& h = cost.dfdx;
    Eigen::SparseMatrix<scalar_t> G = constraints.dfdx.sparseView();
    auto& g = constraints.f;

    return solveDenseQP(H, h, G, g, EInv, mu, lambda, sigma, result);
  };

  SolverStatus solveDenseQP(const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G,
                            const vector_t& g, const vector_t& EInv, const scalar_t mu, const scalar_t lambda, const scalar_t sigma,
                            vector_t& result);
  /**
   * Solve Optimal Control type QP
   *
   * @param x0: Initial state
   * @param dynamics: Dynamics array with N elements(The findal stage do NOT require dynamics).
   * @param cost: State and input cost array with N+1 elements.
   * @param constraints: State-input linear equality constraints. It is optional.
   * @param mu: Lower bound of hessian(H).
   * @param lambda: Upper bound of hessian(H).
   * @param sigma: Upper bound of constraint matrix(G'G).
   * @return SolverStatus
   */
  SolverStatus solveOCPInParallel(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                  const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t& scalingVectors,
                                  const vector_array_t* EInv, const scalar_t mu, const scalar_t lambda, const scalar_t sigma,
                                  const ScalarFunctionQuadraticApproximation& costM /**< [in] For testing only. */,
                                  const VectorFunctionLinearApproximation& constraintsM /**< [in] For testing only. */);

  void calculatePreConditioningFactors(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G, const int iteration,
                                       vector_t& DOut, vector_t& EOut, scalar_t& cOut) const;

  void preConditioningInPlaceInParallel(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                        std::vector<ScalarFunctionQuadraticApproximation>& cost, const int iteration, vector_array_t& DOut,
                                        vector_array_t& EOut, vector_array_t& scalingVectors, scalar_t& cOut,
                                        const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G);

  vector_t HAbsRowSumInParallel(const std::vector<ScalarFunctionQuadraticApproximation>& cost) const;
  /**
   * @brief Scale data array in-place.
   *
   * @param [in] D
   * @param [in] E
   * @param [in] c
   * @param [in, out] dynamics
   * @param [in, out] cost
   * @param [out] scalingVectors
   */
  void scaleDataInPlace(const vector_t& D, const vector_t& E, const scalar_t c, std::vector<VectorFunctionLinearApproximation>& dynamics,
                        std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors) const;
  /**
   * Construct linear constraint matrix
   *
   * @param x0
   * @param dynamics
   * @param constraints
   * @param res
   */
  void getConstraintMatrix(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                           const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                           VectorFunctionLinearApproximation& res) const;

  void getConstraintMatrixSparse(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                 const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                                 Eigen::SparseMatrix<scalar_t>& G, vector_t& g) const;
  /**
   * @brief Get the Cost Matrix object
   *
   * @param x0
   * @param cost
   * @param res
   */
  void getCostMatrix(const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                     ScalarFunctionQuadraticApproximation& res) const;

  /**
   * @brief Get the Cost Matrix Sparse object
   *
   * @param x0
   * @param cost
   * @param H
   * @param h
   */
  void getCostMatrixSparse(const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                           Eigen::SparseMatrix<scalar_t>& H, vector_t& h) const;

  void GGTAbsRowSumInParallel(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                              const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                              vector_t& res);

  /**
   * @brief
   *
   * @param size
   */
  void resize(const OcpSize& size);

  /**
   * @brief Get the Num Stacked Variables object
   *
   * @return int
   */
  int getNumDecisionVariables() const { return numDecisionVariables; }

  int getNumDynamicsConstraints() const { return numDynamicsConstraints; }

  int getNumGeneralEqualityConstraints() const {
    const auto totalNumberOfGeneralEqualityConstraints =
        std::accumulate(ocpSize_.numIneqConstraints.begin(), ocpSize_.numIneqConstraints.end(), 0);
    return totalNumberOfGeneralEqualityConstraints;
  }

  void getStackedSolution(vector_t& res) const;

  void unpackSolution(const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  void packSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution) const;

  void getStateInputTrajectoriesSolution(vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  void descaleSolution(const vector_array_t& D, vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  std::string getBenchmarkingInformationDense() const;
  scalar_t getTotalRunTimeInMilliseconds() const { return parallelizedQPTimer_.getTotalInMilliseconds(); }
  scalar_t getAverageRunTimeInMilliseconds() const { return parallelizedQPTimer_.getAverageInMilliseconds(); }

  const Settings& settings() const { return pipgSettings_; }
  Settings& settings() { return pipgSettings_; }

 private:
  void verifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                   const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   const std::vector<VectorFunctionLinearApproximation>* constraints) const;

  void runParallel(std::function<void(int)> taskFunction);

  void invSqrtInfNorm(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                      const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& scalingVectors, const int N,
                      const std::function<void(vector_t&)>& limitScaling, vector_array_t& D, vector_array_t& E);

  void scaleDataOneStepInPlace(const vector_array_t& D, const vector_array_t& E, const int N,
                               std::vector<VectorFunctionLinearApproximation>& dynamics,
                               std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors);

 private:
  ThreadPool threadPool_;

  Settings pipgSettings_;

  OcpSize ocpSize_;

  // data buffer for parallelized QP
  vector_array_t X_, W_, V_, U_;
  vector_array_t XNew_, UNew_, WNew_;
  scalar_array_t startIndexArray_;

  // Problem size
  int numDecisionVariables;
  int numDynamicsConstraints;

  benchmark::RepeatedTimer denseQPTimer_;
  benchmark::RepeatedTimer parallelizedQPTimer_;

  // Profiling
  benchmark::RepeatedTimer step1v_;
  benchmark::RepeatedTimer step2z_;
  benchmark::RepeatedTimer step3w_;
  benchmark::RepeatedTimer step4CheckConvergence_;
};

vector_t matrixInfNormRows(const Eigen::SparseMatrix<scalar_t>& mat);
vector_t matrixInfNormCols(const Eigen::SparseMatrix<scalar_t>& mat);
vector_t matrixRowwiseAbsSum(const Eigen::SparseMatrix<scalar_t>& mat);
void scaleMatrixInPlace(const vector_t& rowScale, const vector_t& colScale, Eigen::SparseMatrix<scalar_t>& mat);

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
  if (rowScale != nullptr) mat.array().colwise() *= rowScale->array();
  if (colScale != nullptr) mat *= colScale->asDiagonal();
}

}  // namespace ocs2
