
#include "ocs2_pipg/PIPG.h"

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <numeric>

namespace ocs2 {

Pipg::Pipg(pipg::Settings pipgSettings)
    : pipgSettings_(pipgSettings), threadPool_(std::max(pipgSettings.nThreads, size_t(1)) - 1, pipgSettings.threadPriority) {
  Eigen::initParallel();
};

Pipg::SolverStatus Pipg::solveDenseQP(const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G,
                                      const vector_t& g, const vector_t& EInv, const scalar_t mu, const scalar_t lambda,
                                      const scalar_t sigma, vector_t& result) {
  if (const int N = ocpSize_.numStages < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }

  denseQPTimer_.startTimer();

  // Cold start
  vector_t z = vector_t::Zero(H.cols());
  vector_t z_old = vector_t::Zero(H.cols());

  vector_t v = vector_t::Zero(g.rows());
  vector_t w = vector_t::Zero(g.rows());
  vector_t constraintsViolation(g.rows());

  // Iteration number
  size_t k = 0;
  bool isConverged = false;
  scalar_t constraintsViolationInfNorm;
  while (k < settings().maxNumIterations && !isConverged) {
    const scalar_t alpha = 2.0 / ((k + 1.0) * mu + 2.0 * lambda);
    const scalar_t beta = (k + 1) * mu / (2.0 * sigma);

    z_old.swap(z);
    vComputationTimer_.startTimer();
    // v = w + beta * (G * z - g);
    v = -g;
    v.noalias() += G * z;
    v *= beta;
    v += w;
    vComputationTimer_.endTimer();

    zComputationTimer_.startTimer();
    // z = z_old - alpha * (H * z_old + h + G.transpose() * v);
    z = -h;
    z.noalias() -= (H * z_old);
    z.noalias() -= (G.transpose() * v);
    z *= alpha;
    z.noalias() += z_old;
    zComputationTimer_.endTimer();

    wComputationTimer_.startTimer();
    // w = w + beta * (G * z - g);
    w -= beta * g;
    w.noalias() += beta * (G * z);
    wComputationTimer_.endTimer();

    if (k % settings().checkTerminationInterval == 0) {
      convergenceCheckTimer_.startTimer();
      const scalar_t zNorm = z.squaredNorm();

      constraintsViolation.noalias() = G * z;
      constraintsViolation -= g;
      constraintsViolation.cwiseProduct(EInv);
      constraintsViolationInfNorm = constraintsViolation.lpNorm<Eigen::Infinity>();

      const vector_t z_delta = z - z_old;
      const scalar_t z_deltaNorm = z_delta.squaredNorm();
      isConverged = constraintsViolationInfNorm <= settings().absoluteTolerance &&
                    (z_deltaNorm <= settings().relativeTolerance * settings().relativeTolerance * zNorm ||
                     z_deltaNorm <= settings().absoluteTolerance);
      convergenceCheckTimer_.endTimer();
    }

    ++k;
  }
  denseQPTimer_.endTimer();

  Pipg::SolverStatus status = isConverged ? Pipg::SolverStatus::SUCCESS : Pipg::SolverStatus::MAX_ITER;
  if (settings().displayShortSummary) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG-DenseQP " << pipg::toString(status) << " +++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings().maxNumIterations << "\n";
    std::cerr << "Norm of delta primal solution: " << (z - z_old).norm() << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    std::cerr << "Run time: " << denseQPTimer_.getLastIntervalInMilliseconds() << "\n";
  }

  result.swap(z);

  return status;
};

Pipg::SolverStatus Pipg::solveOCPInParallel(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                            const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                                            const std::vector<VectorFunctionLinearApproximation>* constraints,
                                            const vector_array_t& scalingVectors, const vector_array_t* EInv, const scalar_t mu,
                                            const scalar_t lambda, const scalar_t sigma, const ScalarFunctionQuadraticApproximation& costM,
                                            const VectorFunctionLinearApproximation& constraintsM) {
  verifySizes(dynamics, cost, constraints);
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG: solveOCPInParallel] The number of stages cannot be less than 1.");
  }
  if (scalingVectors.size() != N) {
    throw std::runtime_error("[PIPG: solveOCPInParallel] The size of scalingVectors doesn't match the number of stage.");
  }

  // Disable Eigen's internal multithreading
  Eigen::setNbThreads(1);

  parallelizedQPTimer_.startTimer();

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

  scalar_t alpha = 2.0 / (mu + 2.0 * lambda);
  scalar_t beta = mu / (2.0 * sigma);
  scalar_t betaLast = 0;

  // // test
  // vector_t v_test, w_test, z_test, old_z_test, old_old_z_test, old_w_test;
  // scalar_t old_alpha = alpha, old_beta = beta;
  // z_test.setZero(getNumDecisionVariables());
  // w_test.setZero(std::accumulate(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end(), 0));
  // auto& G = constraintsM.dfdx;
  // auto& g = constraintsM.f;
  // auto& H = costM.dfdxx;
  // auto& h = costM.dfdx;
  // // test end

  size_t k = 0;
  std::atomic_int timeIndex{1}, finishedTaskCounter{0};
  std::atomic_bool keepRunning{true}, shouldWait{true};
  bool isConverged = false;

  std::mutex mux;
  std::condition_variable iterationFinished;
  std::vector<int> threadsWorkloadCounter(threadPool_.numThreads() + 1, 0);

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
        threadsWorkloadCounter[workerId]++;

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
        alpha = 2.0 / ((static_cast<scalar_t>(k) + 1.0) * mu + 2.0 * lambda);
        beta = (static_cast<scalar_t>(k) + 1.0) * mu / (2.0 * sigma);

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

        // /**
        //  * *********************************
        //  * ************ Test begin *********
        //  * *********************************
        //  */
        // old_old_z_test = old_z_test;
        // old_z_test = z_test;
        // old_w_test = w_test;

        // v_test = w_test + old_beta * (G * z_test - g);
        // z_test = z_test - old_alpha * (H * z_test + h + G.transpose() * v_test);
        // w_test = w_test + old_beta * (G * z_test - g);

        // old_alpha = alpha;
        // old_beta = beta;

        // vector_t V_all(V_.size() * ocpSize_.numStates.front()), W_all(W_.size() * ocpSize_.numStates.front()),
        //     X_all(X_.size() * ocpSize_.numStates.front()), U_all(U_.size() * ocpSize_.numInputs.front()),
        //     Z_all(getNumDecisionVariables());
        // int curRow = 0;
        // for (int i = 0; i < N; i++) {
        //   const auto nx = ocpSize_.numStates[i + 1];
        //   V_all.segment(curRow, nx) = V_[i];
        //   W_all.segment(curRow, nx) = W_[i];
        //   curRow += nx;
        // }
        // getStackedSolution(Z_all);

        // if (!V_all.isApprox(v_test)) {
        //   std::cerr << "k: " << k << "\n";
        //   std::cerr << "v_test: " << v_test.transpose() << "\n";
        //   std::cerr << "v_all: " << V_all.transpose() << "\n";
        //   std::cerr << "diff:v " << (V_all - v_test).transpose() << std::endl;
        // }
        // if (!Z_all.isApprox(z_test)) {
        //   std::cerr << "k: " << k << "\n";
        //   std::cerr << "z_test: " << z_test.transpose() << "\n";
        //   std::cerr << "z_all: " << Z_all.transpose() << "\n";
        //   for (auto& v : X_) std::cerr << v.transpose() << "\n";
        //   std::cerr << "\n";
        //   for (auto& v : U_) std::cerr << v.transpose() << "\n";
        //   std::cerr << "diff:z " << (Z_all - z_test).transpose() << "\n" << std::endl;
        // }
        // if (k != 0 && !W_all.isApprox(old_w_test)) {
        //   std::cerr << "k: " << k << "\n";
        //   std::cerr << "w_test: " << w_test.transpose() << "\n";
        //   std::cerr << "w_all: " << W_all.transpose() << "\n";
        //   std::cerr << "diff:w " << (W_all - w_test).transpose() << std::endl;
        // }
        // if (k != 0 && k % settings().checkTerminationInterval == 0 &&
        //     (std ::abs((EInv.asDiagonal() * (G * old_z_test - g)).lpNorm<Eigen::Infinity>() - constraintsViolationInfNorm) >
        //      settings().absoluteTolerance)) {
        //   std::cerr << "k: " << k << "\n";
        //   std::cerr << "base: " << (G * old_z_test - g).cwiseAbs().maxCoeff() << "\n";
        //   std::cerr << "my: " << constraintsViolationInfNorm << "\n\n";
        // }
        // if (k != 0 && k % settings().checkTerminationInterval == 0 &&
        //     (std::abs((old_z_test - old_old_z_test).squaredNorm() - solutionSSE) > 1e-12 ||
        //      std::abs(old_z_test.squaredNorm() - solutionSquaredNorm) > 1e-12)) {
        //   std::cerr << "k: " << k << "\n";
        //   std::cerr << "solutionSSE diff: " << std::scientific << std::abs((old_z_test - old_old_z_test).squaredNorm() - solutionSSE)
        //             << "\n";
        //   std::cerr << "solutionSSE: " << solutionSSE << "\n";
        //   std::cerr << "base: " << (old_z_test - old_old_z_test).squaredNorm() << "\n";
        //   std::cerr << "solutionSquaredNorm diff: " << std::scientific << std::abs(old_z_test.squaredNorm() - solutionSquaredNorm) <<
        //   "\n"; std::cerr << "solutionSquaredNorm: " << solutionSquaredNorm << "\n"; std::cerr << "base: " << old_z_test.squaredNorm() <<
        //   "\n\n";
        // }
        // /**
        //  * *********************************
        //  * ************ Test end ***********
        //  * *********************************
        //  */

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
  runParallel(std::move(updateVariablesTask));

  parallelizedQPTimer_.endTimer();

  Pipg::SolverStatus status = isConverged ? Pipg::SolverStatus::SUCCESS : Pipg::SolverStatus::MAX_ITER;
  if (settings().displayShortSummary) {
    scalar_t totalTasks = std::accumulate(threadsWorkloadCounter.cbegin(), threadsWorkloadCounter.cend(), 0.0);
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG-Parallel " << pipg::toString(status) << " +++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings().maxNumIterations << "\n";
    std::cerr << "Norm of delta primal solution: " << std::sqrt(solutionSSE) << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    std::cerr << "Thread workload(ID: # of finished tasks): ";
    for (int i = 0; i < threadsWorkloadCounter.size(); i++) {
      std::cerr << i << ": " << threadsWorkloadCounter[i] << "(" << static_cast<scalar_t>(threadsWorkloadCounter[i]) / totalTasks * 100.0
                << "%) ";
    }
    std::cerr << "\nRun time: " << parallelizedQPTimer_.getLastIntervalInMilliseconds() << "\n\n";
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  return status;
};

void Pipg::resize(const OcpSize& ocpSize) {
  if (ocpSize_ == ocpSize) {
    return;
  }
  verifyOcpSize(ocpSize);

  ocpSize_ = ocpSize;
  const int N = ocpSize_.numStages;

  numDecisionVariables_ = std::accumulate(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end(), 0);
  numDecisionVariables_ += std::accumulate(ocpSize_.numInputs.begin(), ocpSize_.numInputs.end(), 0);
  numDynamicsConstraints_ = std::accumulate(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end(), 0);

  X_.resize(N + 1);
  W_.resize(N);
  V_.resize(N);
  U_.resize(N);
  XNew_.resize(N + 1);
  UNew_.resize(N);
  WNew_.resize(N);
}

void Pipg::getStackedSolution(vector_t& res) const {
  packSolution(X_, U_, res);
}

void Pipg::unpackSolution(const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
                          vector_array_t& uTrajectory) const {
  const int N = ocpSize_.numStages;
  xTrajectory.resize(N + 1);
  uTrajectory.resize(N);

  xTrajectory.front() = x0;

  int curRow = 0;
  for (int i = 0; i < N; i++) {
    const auto nx = ocpSize_.numStates[i + 1];
    const auto nu = ocpSize_.numInputs[i];
    xTrajectory[i + 1] = stackedSolution.segment(curRow + nu, nx);
    uTrajectory[i] = stackedSolution.segment(curRow, nu);

    curRow += nx + nu;
  }
}

void Pipg::packSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution) const {
  stackedSolution.resize(getNumDecisionVariables());

  int curRow = 0;
  for (int i = 0; i < ocpSize_.numStages; i++) {
    const auto nx = ocpSize_.numStates[i + 1];
    const auto nu = ocpSize_.numInputs[i];
    stackedSolution.segment(curRow, nx + nu) << uTrajectory[i], xTrajectory[i + 1];
    curRow += nx + nu;
  }
}

void Pipg::descaleSolution(const vector_array_t& D, vector_array_t& xTrajectory, vector_array_t& uTrajectory) const {
  const int N = ocpSize_.numStages;
  if (D.size() != xTrajectory.size() + uTrajectory.size() - 1) {
    throw std::runtime_error("[PIPG]::descaleSolution - Size doesn't match.");
  }

  for (int k = 0; k < N; k++) {
    uTrajectory[k].array() *= D[2 * k].array();
    xTrajectory[k + 1].array() *= D[2 * k + 1].array();
  }
}

void Pipg::getStateInputTrajectoriesSolution(vector_array_t& xTrajectory, vector_array_t& uTrajectory) const {
  xTrajectory.resize(X_.size());
  uTrajectory.resize(U_.size());

  std::copy(X_.begin(), X_.end(), xTrajectory.begin());
  std::copy(U_.begin(), U_.end(), uTrajectory.begin());
}

void Pipg::runParallel(std::function<void(int)> taskFunction) {
  threadPool_.runParallel(std::move(taskFunction), settings().nThreads);
}

void Pipg::verifySizes(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                       const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                       const std::vector<VectorFunctionLinearApproximation>* constraints) const {
  if (dynamics.size() != ocpSize_.numStages) {
    throw std::runtime_error("[PIPG] Inconsistent size of dynamics: " + std::to_string(dynamics.size()) + " with " +
                             std::to_string(ocpSize_.numStages) + " number of stages.");
  }
  if (cost.size() != ocpSize_.numStages + 1) {
    throw std::runtime_error("[PIPG] Inconsistent size of cost: " + std::to_string(cost.size()) + " with " +
                             std::to_string(ocpSize_.numStages + 1) + " nodes.");
  }
  if (constraints != nullptr) {
    if (constraints->size() != ocpSize_.numStages + 1) {
      throw std::runtime_error("[PIPG] Inconsistent size of constraints: " + std::to_string(constraints->size()) + " with " +
                               std::to_string(ocpSize_.numStages + 1) + " nodes.");
    }
  }
}

void Pipg::verifyOcpSize(const OcpSize& ocpSize) const {
  auto isNotEmpty = [](const std::vector<int>& v) { return std::any_of(v.cbegin(), v.cend(), [](int s) { return s != 0; }); };

  if (isNotEmpty(ocpSize.numInputBoxConstraints)) {
    throw std::runtime_error("[Pipg::verifyOcpSize] PIPG solver does not support input box constraints.");
  }
  if (isNotEmpty(ocpSize.numStateBoxConstraints)) {
    throw std::runtime_error("[Pipg::verifyOcpSize] PIPG solver does not support state box constraints.");
  }
  if (isNotEmpty(ocpSize.numInputBoxSlack)) {
    throw std::runtime_error("[Pipg::verifyOcpSize] PIPG solver does not support input slack variables.");
  }
  if (isNotEmpty(ocpSize.numStateBoxSlack)) {
    throw std::runtime_error("[Pipg::verifyOcpSize] PIPG solver does not support state slack variables.");
  }
  if (isNotEmpty(ocpSize.numIneqSlack)) {
    throw std::runtime_error("[Pipg::verifyOcpSize] PIPG solver does not support inequality slack variables.");
  }
}

int Pipg::getNumGeneralEqualityConstraints() const {
  const auto totalNumberOfGeneralEqualityConstraints =
      std::accumulate(ocpSize_.numIneqConstraints.begin(), ocpSize_.numIneqConstraints.end(), 0);
  return totalNumberOfGeneralEqualityConstraints;
}

std::string Pipg::getBenchmarkingInformationDense() const {
  const auto step1v = vComputationTimer_.getTotalInMilliseconds();
  const auto step2z = zComputationTimer_.getTotalInMilliseconds();
  const auto step3w = wComputationTimer_.getTotalInMilliseconds();
  const auto step4CheckConvergence = convergenceCheckTimer_.getTotalInMilliseconds();

  const auto benchmarkTotal = step1v + step2z + step3w + step4CheckConvergence;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << vComputationTimer_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PIPG Dense Benchmarking\t     :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tvComputation         :\t" << vComputationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << step1v / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tzComputation         :\t" << zComputationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << step2z / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\twComputation         :\t" << wComputationTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << step3w / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tCheckConvergence     :\t" << convergenceCheckTimer_.getAverageInMilliseconds() << " [ms] \t\t("
               << step4CheckConvergence / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

Pipg::~Pipg() {
  if (settings().displayShortSummary) {
    std::cerr << getBenchmarkingInformationDense() << std::endl;
  }
}
}  // namespace ocs2
