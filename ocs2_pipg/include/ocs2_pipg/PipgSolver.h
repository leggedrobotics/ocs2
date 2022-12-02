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

#pragma once

#include <string>

#include <Eigen/Sparse>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_oc/oc_problem/OcpSize.h>

#include "ocs2_pipg/PipgSettings.h"
#include "ocs2_pipg/PipgSolverStatus.h"

namespace ocs2 {

class PipgSolver {
 public:
  /**
   * @brief Construct a new PIPG with pipg setting object.
   *
   * @param Settings: PIPG setting
   */
  explicit PipgSolver(pipg::Settings settings);

  /** Destructor */
  ~PipgSolver();

  /**
   * @brief Solve generic QP problem.
   *
   * @param H: Cost hessian.
   * @param h: Cost linear component.
   * @param G: Linear equality constraint matrix.
   * @param g: Linear equality constraint values
   * @param EInv: Inverse of the scaling matrix E. Used to calculate un-sacled termination criteria.
   * @param mu: the lower bound of the cost hessian H.
   * @param lambda: the upper bound of the cost hessian H.
   * @param sigma: the upper bound of \f$ G^TG \f$
   * @param result: Stacked result.
   * @return SolverStatus
   */
  pipg::SolverStatus solveDenseQP(const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G,
                                  const vector_t& g, const vector_t& EInv, const scalar_t mu, const scalar_t lambda, const scalar_t sigma,
                                  vector_t& result);

  /**
   * @brief Solve Optimal Control type QP in parallel.
   *
   * @param x0 Initial state
   * @param dynamics: Dynamics array.
   * @param cost: Cost array.
   * @param constraints: Constraints array. Pass nullptr for an unconstrained problem.
   * @param scalingVectors Vector representatoin for the identity parts of the dynamics constraints inside the constraint matrix. After
   * scaling, they become arbitrary diagonal matrices. Pass nullptr to get them filled with identity matrices.
   * @param EInv Inverse of the scaling factor E. Used to calculate un-sacled termination criteria.
   * @param mu: the lower bound of the cost hessian H.
   * @param lambda: the upper bound of the cost hessian H.
   * @param sigma: the upper bound of \f$ G^TG \f$
   * @param costM For testing only. Can be removed.
   * @param constraintsM For testing only. Can be removed.
   * @return SolverStatus
   */
  pipg::SolverStatus solveOCPInParallel(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                        const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                        const std::vector<VectorFunctionLinearApproximation>* constraints,
                                        const vector_array_t& scalingVectors, const vector_array_t* EInv, const scalar_t mu,
                                        const scalar_t lambda, const scalar_t sigma, const ScalarFunctionQuadraticApproximation& costM,
                                        const VectorFunctionLinearApproximation& constraintsM);

  void resize(const OcpSize& size);

  int getNumDecisionVariables() const { return numDecisionVariables_; }

  int getNumDynamicsConstraints() const { return numDynamicsConstraints_; }

  int getNumGeneralEqualityConstraints() const;

  void getStackedSolution(vector_t& res) const { packSolution(X_, U_, res); }

  void unpackSolution(const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  void packSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution) const;

  void getStateInputTrajectoriesSolution(vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  void descaleSolution(const vector_array_t& D, vector_array_t& xTrajectory, vector_array_t& uTrajectory) const;

  std::string getBenchmarkingInformationDense() const;
  scalar_t getTotalRunTimeInMilliseconds() const { return parallelizedQPTimer_.getTotalInMilliseconds(); }
  scalar_t getAverageRunTimeInMilliseconds() const { return parallelizedQPTimer_.getAverageInMilliseconds(); }

  const pipg::Settings& settings() const { return settings_; }
  const OcpSize& size() const { return ocpSize_; }
  ThreadPool& getThreadPool() { return threadPool_; }

 private:
  void verifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                   const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                   const std::vector<VectorFunctionLinearApproximation>* constraints) const;

  void verifyOcpSize(const OcpSize& ocpSize) const;

  void runParallel(std::function<void(int)> taskFunction) { threadPool_.runParallel(std::move(taskFunction), settings().nThreads); }

 private:
  const pipg::Settings settings_;
  ThreadPool threadPool_;
  OcpSize ocpSize_;

  // Data buffer for parallelized QP
  vector_array_t X_, W_, V_, U_;
  vector_array_t XNew_, UNew_, WNew_;

  // Problem size
  int numDecisionVariables_;
  int numDynamicsConstraints_;

  // Profiling
  // Dense PIPG
  benchmark::RepeatedTimer denseQPTimer_;
  benchmark::RepeatedTimer parallelizedQPTimer_;

  // Parallel PIPG
  benchmark::RepeatedTimer vComputationTimer_;
  benchmark::RepeatedTimer zComputationTimer_;
  benchmark::RepeatedTimer wComputationTimer_;
  benchmark::RepeatedTimer convergenceCheckTimer_;
};
}  // namespace ocs2
