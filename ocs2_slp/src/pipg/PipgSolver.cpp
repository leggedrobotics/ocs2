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

#include "ocs2_slp/pipg/PipgSolver.h"

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <numeric>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PipgSolver::PipgSolver(pipg::Settings settings) : settings_(std::move(settings)) {
  Eigen::initParallel();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
pipg::SolverStatus PipgSolver::solve(ThreadPool& threadPool, const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                     const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                     const std::vector<VectorFunctionLinearApproximation>* constraints,
                                     const vector_array_t& scalingVectors, const vector_array_t* EInv, const pipg::PipgBounds& pipgBounds,
                                     vector_array_t& xTrajectory, vector_array_t& uTrajectory) {
  verifySizes(dynamics, cost, constraints);
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PipgSolver::solve] The number of stages cannot be less than 1.");
  }
  if (scalingVectors.size() != N) {
    throw std::runtime_error("[PipgSolver::solve] The size of scalingVectors doesn't match the number of stage.");
  }

  // Disable Eigen's internal multithreading
  Eigen::setNbThreads(1);

  vector_array_t primalResidualArray(N);
  scalar_array_t constraintsViolationInfNormArray(N);
  scalar_t constraintsViolationInfNorm;

  scalar_t solutionSSE, solutionSquaredNorm;
  scalar_array_t solutionSEArray(N);
  scalar_array_t solutionSquaredNormArray(N);

  // initial state
  X_[0] = x0;
  XNew_[0] = x0;
  // cold start
  for (int t = 0; t < N; t++) {
    X_[t + 1].setZero(dynamics[t].dfdx.rows());
    U_[t].setZero(dynamics[t].dfdu.cols());
    W_[t].setZero(dynamics[t].dfdx.rows());
    // WNew_ will NOT be filled, but will be swapped to W_ in iteration 0. Thus, initialize WNew_ here.
    WNew_[t].setZero(dynamics[t].dfdx.rows());
  }

  scalar_t alpha = pipgBounds.primalStepSize(0);
  scalar_t beta = pipgBounds.primalStepSize(0);
  scalar_t betaLast = 0;

  size_t k = 0;
  std::atomic_int timeIndex{1}, finishedTaskCounter{0};
  std::atomic_bool keepRunning{true}, shouldWait{true};
  bool isConverged = false;

  std::mutex mux;
  std::condition_variable iterationFinished;
  std::vector<int> threadsWorkloadCounter(threadPool.numThreads() + 1U, 0);

  auto updateVariablesTask = [&](int workerId) {
    int t;
    int workerOrder;

    while (keepRunning) {
      // Reset workerOrder in case all tasks have been assigned and some workers cannot enter the following while loop, keeping the
      // workerOrder from previous iterations.
      workerOrder = 0;
      while ((t = timeIndex++) <= N) {
        if (t == N) {
          std::lock_guard<std::mutex> lk(mux);
          shouldWait = true;
        }
        // Multi-thread performance analysis
        ++threadsWorkloadCounter[workerId];

        // PIPG algorithm
        const auto& A = dynamics[t - 1].dfdx;
        const auto& B = dynamics[t - 1].dfdu;
        const auto& C = scalingVectors[t - 1];
        const auto& b = dynamics[t - 1].f;

        const auto& R = cost[t - 1].dfduu;
        const auto& Q = cost[t].dfdxx;
        const auto& P = cost[t - 1].dfdux;
        const auto& q = cost[t].dfdx;
        const auto& r = cost[t - 1].dfdu;

        if (k != 0) {
          // Update W of the iteration k - 1. Move the update of W to the front of the calculation of V to prevent data race.
          // vector_t primalResidual = C * X_[t] - A * X_[t - 1] - B * U_[t - 1] - b;
          primalResidualArray[t - 1] = -b;
          primalResidualArray[t - 1].array() += C.array() * X_[t].array();
          primalResidualArray[t - 1].noalias() -= A * X_[t - 1];
          primalResidualArray[t - 1].noalias() -= B * U_[t - 1];
          if (EInv != nullptr) {
            constraintsViolationInfNormArray[t - 1] = (*EInv)[t - 1].cwiseProduct(primalResidualArray[t - 1]).lpNorm<Eigen::Infinity>();
          } else {
            constraintsViolationInfNormArray[t - 1] = primalResidualArray[t - 1].lpNorm<Eigen::Infinity>();
          }

          WNew_[t - 1] = W_[t - 1] + betaLast * primalResidualArray[t - 1];

          // What stored in UNew and XNew is the solution of iteration k - 2 and what stored in U and X is the solution of iteration k
          // - 1. By convention, iteration starts from 0 and the solution of iteration -1 is the initial value. Reuse UNew and XNew
          // memory to store the difference between the last solution and the one before last solution.
          UNew_[t - 1] -= U_[t - 1];
          XNew_[t] -= X_[t];

          solutionSEArray[t - 1] = UNew_[t - 1].squaredNorm() + XNew_[t].squaredNorm();
          solutionSquaredNormArray[t - 1] = U_[t - 1].squaredNorm() + X_[t].squaredNorm();
        }

        // V_[t - 1] = W_[t - 1] + (beta + betaLast) * (C * X_[t] - A * X_[t - 1] - B * U_[t - 1] - b);
        V_[t - 1] = W_[t - 1] - (beta + betaLast) * b;
        V_[t - 1].array() += (beta + betaLast) * C.array() * X_[t].array();
        V_[t - 1].noalias() -= (beta + betaLast) * (A * X_[t - 1]);
        V_[t - 1].noalias() -= (beta + betaLast) * (B * U_[t - 1]);

        // UNew_[t - 1] = U_[t - 1] - alpha * (R * U_[t - 1] + P * X_[t - 1] + r - B.transpose() * V_[t - 1]);
        UNew_[t - 1] = U_[t - 1] - alpha * r;
        UNew_[t - 1].noalias() -= alpha * (R * U_[t - 1]);
        UNew_[t - 1].noalias() -= alpha * (P * X_[t - 1]);
        UNew_[t - 1].noalias() += alpha * (B.transpose() * V_[t - 1]);

        // XNew_[t] = X_[t] - alpha * (Q * X_[t] + q + C * V_[t - 1]);
        XNew_[t] = X_[t] - alpha * q;
        XNew_[t].array() -= alpha * C.array() * V_[t - 1].array();
        XNew_[t].noalias() -= alpha * (Q * X_[t]);

        if (t != N) {
          const auto& ANext = dynamics[t].dfdx;
          const auto& BNext = dynamics[t].dfdu;
          const auto& CNext = scalingVectors[t];
          const auto& bNext = dynamics[t].f;

          // dfdux
          const auto& PNext = cost[t].dfdux;

          // vector_t VNext = W_[t] + (beta + betaLast) * (CNext * X_[t + 1] - ANext * X_[t] - BNext * U_[t] - bNext);
          vector_t VNext = W_[t] - (beta + betaLast) * bNext;
          VNext.array() += (beta + betaLast) * CNext.array() * X_[t + 1].array();
          VNext.noalias() -= (beta + betaLast) * (ANext * X_[t]);
          VNext.noalias() -= (beta + betaLast) * (BNext * U_[t]);

          XNew_[t].noalias() += alpha * (ANext.transpose() * VNext);
          // Add dfdxu * du if it is not the final state.
          XNew_[t].noalias() -= alpha * (PNext.transpose() * U_[t]);
        }

        workerOrder = ++finishedTaskCounter;
      }

      if (workerOrder != N) {
        std::unique_lock<std::mutex> lk(mux);
        iterationFinished.wait(lk, [&shouldWait] { return !shouldWait; });
        lk.unlock();
      } else {
        betaLast = beta;
        // Adaptive step size
        beta = pipgBounds.dualStepSize(k);
        alpha = pipgBounds.primalStepSize(k);

        if (k != 0 && k % settings().checkTerminationInterval == 0) {
          constraintsViolationInfNorm =
              *(std::max_element(constraintsViolationInfNormArray.begin(), constraintsViolationInfNormArray.end()));

          solutionSSE = std::accumulate(solutionSEArray.begin(), solutionSEArray.end(), 0.0);
          solutionSquaredNorm = std::accumulate(solutionSquaredNormArray.begin(), solutionSquaredNormArray.end(), 0.0);

          isConverged = constraintsViolationInfNorm <= settings().absoluteTolerance &&
                        (solutionSSE <= settings().relativeTolerance * settings().relativeTolerance * solutionSquaredNorm ||
                         solutionSSE <= settings().absoluteTolerance);

          keepRunning = k < settings().maxNumIterations && !isConverged;
        }

        XNew_.swap(X_);
        UNew_.swap(U_);
        WNew_.swap(W_);

        ++k;
        finishedTaskCounter = 0;
        timeIndex = 1;
        {
          std::lock_guard<std::mutex> lk(mux);
          shouldWait = false;
        }
        iterationFinished.notify_all();
      }
    }
  };
  threadPool.runParallel(std::move(updateVariablesTask), threadPool.numThreads() + 1U);

  xTrajectory = X_;
  uTrajectory = U_;
  const auto status = isConverged ? pipg::SolverStatus::SUCCESS : pipg::SolverStatus::MAX_ITER;

  if (settings().displayShortSummary) {
    scalar_t totalTasks = std::accumulate(threadsWorkloadCounter.cbegin(), threadsWorkloadCounter.cend(), 0.0);
    std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG +++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Solver status: " << pipg::toString(status) << "\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings().maxNumIterations << "\n";
    std::cerr << "Norm of delta primal solution: " << std::sqrt(solutionSSE) << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    std::cerr << "Thread workload(ID: # of finished tasks): ";
    for (int i = 0; i < threadsWorkloadCounter.size(); i++) {
      std::cerr << i << ": " << threadsWorkloadCounter[i] << "(" << static_cast<scalar_t>(threadsWorkloadCounter[i]) / totalTasks * 100.0
                << "%) ";
    }
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  return status;
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PipgSolver::resize(const OcpSize& ocpSize) {
  if (ocpSize_ == ocpSize) {
    return;
  }
  verifyOcpSize(ocpSize);

  ocpSize_ = ocpSize;
  const int N = ocpSize_.numStages;

  numDecisionVariables_ = std::accumulate(std::next(ocpSize_.numStates.begin()), ocpSize_.numStates.end(), 0);
  numDecisionVariables_ += std::accumulate(ocpSize_.numInputs.begin(), ocpSize_.numInputs.end(), 0);
  numDynamicsConstraints_ = std::accumulate(std::next(ocpSize_.numStates.begin()), ocpSize_.numStates.end(), 0);

  X_.resize(N + 1);
  W_.resize(N);
  V_.resize(N);
  U_.resize(N);
  XNew_.resize(N + 1);
  UNew_.resize(N);
  WNew_.resize(N);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PipgSolver::verifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                             const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                             const std::vector<VectorFunctionLinearApproximation>* constraints) const {
  if (dynamics.size() != ocpSize_.numStages) {
    throw std::runtime_error("[PipgSolver::verifySizes] Inconsistent size of dynamics: " + std::to_string(dynamics.size()) + " with " +
                             std::to_string(ocpSize_.numStages) + " number of stages.");
  }
  if (cost.size() != ocpSize_.numStages + 1) {
    throw std::runtime_error("[PipgSolver::verifySizes] Inconsistent size of cost: " + std::to_string(cost.size()) + " with " +
                             std::to_string(ocpSize_.numStages + 1) + " nodes.");
  }
  if (constraints != nullptr) {
    if (constraints->size() != ocpSize_.numStages + 1) {
      throw std::runtime_error("[PipgSolver::verifySizes] Inconsistent size of constraints: " + std::to_string(constraints->size()) +
                               " with " + std::to_string(ocpSize_.numStages + 1) + " nodes.");
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void PipgSolver::verifyOcpSize(const OcpSize& ocpSize) const {
  auto isNotEmpty = [](const std::vector<int>& v) { return std::any_of(v.cbegin(), v.cend(), [](int s) { return s != 0; }); };

  if (isNotEmpty(ocpSize.numInputBoxConstraints)) {
    throw std::runtime_error("[PipgSolver::verifyOcpSize] PIPG solver does not support input box constraints.");
  }
  if (isNotEmpty(ocpSize.numStateBoxConstraints)) {
    throw std::runtime_error("[PipgSolver::verifyOcpSize] PIPG solver does not support state box constraints.");
  }
  if (isNotEmpty(ocpSize.numInputBoxSlack)) {
    throw std::runtime_error("[PipgSolver::verifyOcpSize] PIPG solver does not support input slack variables.");
  }
  if (isNotEmpty(ocpSize.numStateBoxSlack)) {
    throw std::runtime_error("[PipgSolver::verifyOcpSize] PIPG solver does not support state slack variables.");
  }
  if (isNotEmpty(ocpSize.numIneqSlack)) {
    throw std::runtime_error("[PipgSolver::verifyOcpSize] PIPG solver does not support inequality slack variables.");
  }
}

}  // namespace ocs2
