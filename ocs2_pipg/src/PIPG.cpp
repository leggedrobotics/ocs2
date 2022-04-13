
#include "ocs2_pipg/PIPG.h"

#include <iostream>

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
  vector_t z_old;

  scalar_t alpha, beta;
  vector_t v, w;

  w.setZero(g.rows());

  // Iteration number
  size_t k = 0;
  bool isConverged = false;
  scalar_t constraintsViolationInfNorm;
  scalar_t zNorm;
  while (k < settings().maxNumIterations && !isConverged) {
    alpha = 2.0 / ((k + 1.0) * mu + 2.0 * lambda);
    beta = (k + 1) * mu / (2.0 * sigma);

    step1v_.startTimer();
    z_old = z;
    // v = w + beta * (G * z - g);
    v = w - beta * g;
    v.noalias() += beta * (G * z);
    step1v_.endTimer();

    step2z_.startTimer();
    // z = z_old - alpha * (H * z_old + h + G.transpose() * v);
    z = z_old - alpha * h;
    z.noalias() -= alpha * (H * z_old);
    z.noalias() -= alpha * (G.transpose() * v);
    step2z_.endTimer();

    step3w_.startTimer();
    // w = w + beta * (G * z - g);
    w -= beta * g;
    w.noalias() += beta * (G * z);
    step3w_.endTimer();

    if (k % settings().checkTerminationInterval == 0) {
      step4CheckConvergence_.startTimer();
      zNorm = z.squaredNorm() < 0.1 ? 1.0 : z.squaredNorm();
      constraintsViolationInfNorm = (EInv.asDiagonal() * (G * z - g)).lpNorm<Eigen::Infinity>();
      isConverged = constraintsViolationInfNorm <= settings().absoluteTolerance &&
                    (z - z_old).squaredNorm() <= settings().relativeTolerance * settings().relativeTolerance * zNorm;
      step4CheckConvergence_.endTimer();
    }

    k++;
  }
  denseQPTimer_.endTimer();
  result = z;

  if (settings().displayShortSummary) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG-DenseQP " << (isConverged ? "Success" : "Fail") << " +++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings().maxNumIterations << "\n";
    std::cerr << "Relative change of primal solution : " << std::sqrt((z - z_old).squaredNorm() / zNorm) << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    std::cerr << "Run time: " << denseQPTimer_.getLastIntervalInMilliseconds() << "\n";
  }
  return Pipg::SolverStatus::SUCCESS;
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
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
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
        const auto& hx = cost[t].dfdx;
        const auto& hu = cost[t - 1].dfdu;

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

        // UNew_[t - 1] = U_[t - 1] - alpha * (R * U_[t - 1] + P * X_[t - 1] + hu - B.transpose() * V_[t - 1]);
        UNew_[t - 1] = U_[t - 1] - alpha * hu;
        UNew_[t - 1].noalias() -= alpha * (R * U_[t - 1]);
        UNew_[t - 1].noalias() -= alpha * (P * X_[t - 1]);
        UNew_[t - 1].noalias() += alpha * (B.transpose() * V_[t - 1]);

        // XNew_[t] = X_[t] - alpha * (Q * X_[t] + hx + C * V_[t - 1]);
        XNew_[t] = X_[t] - alpha * hx;
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
          solutionSquaredNorm = solutionSquaredNorm < 0.1 ? 1.0 : solutionSquaredNorm;

          isConverged = constraintsViolationInfNorm <= settings().absoluteTolerance &&
                        solutionSSE <= settings().relativeTolerance * settings().relativeTolerance * solutionSquaredNorm;

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

  if (settings().displayShortSummary) {
    scalar_t totalTasks = std::accumulate(threadsWorkloadCounter.cbegin(), threadsWorkloadCounter.cend(), 0.0);
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ PIPG-Parallel " << (isConverged ? "Success" : "Fail") << " +++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings().maxNumIterations << "\n";
    std::cerr << "Relative change of primal solution : " << std::sqrt(solutionSSE / solutionSquaredNorm) << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    std::cerr << "Thread workload(ID: # of finished tasks): ";
    for (int i = 0; i < threadsWorkloadCounter.size(); i++) {
      std::cerr << i << ": " << threadsWorkloadCounter[i] << "(" << static_cast<scalar_t>(threadsWorkloadCounter[i]) / totalTasks * 100.0
                << "%) ";
    }
    std::cerr << "\nRun time: " << parallelizedQPTimer_.getLastIntervalInMilliseconds() << "\n\n";
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  return Pipg::SolverStatus::SUCCESS;
};

void Pipg::calculatePreConditioningFactors(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G,
                                           const int iteration, vector_t& DOut, vector_t& EOut, scalar_t& cOut) const {
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

void Pipg::invSqrtInfNorm(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                          const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& scalingVectors, const int N,
                          const std::function<void(vector_t&)>& limitScaling, vector_array_t& D, vector_array_t& E) {
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

  runParallel(std::move(task));

  procedure(D[2 * N - 1], matrixInfNormCols(cost[N].dfdxx, scalingVectors[N - 1].asDiagonal().toDenseMatrix()));
}

void Pipg::scaleDataOneStepInPlace(const vector_array_t& D, const vector_array_t& E, const int N,
                                   std::vector<VectorFunctionLinearApproximation>& dynamics,
                                   std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors) {
  scaleMatrixInPlace(&(D[0]), &(D[0]), cost.front().dfduu);
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdu);
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdux);

  std::atomic_int timeStamp{1};
  auto scaleCost = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      auto& Q = cost[k].dfdxx;
      auto& R = cost[k].dfduu;
      auto& P = cost[k].dfdux;
      auto& q = cost[k].dfdx;
      auto& r = cost[k].dfdu;

      scaleMatrixInPlace(&(D[2 * k - 1]), &(D[2 * k - 1]), Q);
      scaleMatrixInPlace(&(D[2 * k - 1]), nullptr, q);
      scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k - 1]), P);

      scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k]), R);
      scaleMatrixInPlace(&(D[2 * k]), nullptr, r);
    }
  };

  runParallel(std::move(scaleCost));

  scaleMatrixInPlace(&(D.back()), &(D.back()), cost[N].dfdxx);
  scaleMatrixInPlace(&(D.back()), nullptr, cost[N].dfdx);

  // Scale G & g
  auto& B_0 = dynamics[0].dfdu;
  auto& C_0 = scalingVectors[0];
  /**
   * \tilde{B} = E * B * D
   * scaling = E * I * D
   * \tilde{g} = E * g
   */
  scaleMatrixInPlace(&(E[0]), &(D[0]), B_0);
  C_0.array() *= E[0].array() * D[1].array();

  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].dfdx);
  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].f);

  timeStamp = 1;
  auto scaleConstraints = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      auto& A_k = dynamics[k].dfdx;
      auto& B_k = dynamics[k].dfdu;
      auto& b_k = dynamics[k].f;
      auto& C_k = scalingVectors[k];

      scaleMatrixInPlace(&(E[k]), &(D[2 * k - 1]), A_k);
      scaleMatrixInPlace(&(E[k]), &(D[2 * k]), B_k);
      C_k.array() *= E[k].array() * D[2 * k + 1].array();

      scaleMatrixInPlace(&(E[k]), nullptr, b_k);
    }
  };

  runParallel(std::move(scaleConstraints));
}

void Pipg::preConditioningInPlaceInParallel(const vector_t& x0, std::vector<VectorFunctionLinearApproximation>& dynamics,
                                            std::vector<ScalarFunctionQuadraticApproximation>& cost, const int iteration,
                                            vector_array_t& DOut, vector_array_t& EOut, vector_array_t& scalingVectors, scalar_t& cOut,
                                            const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                                            const Eigen::SparseMatrix<scalar_t>& G) {
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
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
    DOut[2 * i].setOnes(ocpSize_.numInputs[i]);
    DOut[2 * i + 1].setOnes(ocpSize_.numStates[i + 1]);
    EOut[i].setOnes(ocpSize_.numStates[i + 1]);
    scalingVectors[i].setOnes(ocpSize_.numStates[i + 1]);
  }

  // /**
  //  * Test start
  //  */
  // Eigen::SparseMatrix<scalar_t> HTest = H;
  // vector_t hTest = h;
  // Eigen::SparseMatrix<scalar_t> GTest = G;
  // /**
  //  * Test end
  //  */

  vector_array_t D(2 * N), E(N);
  for (int i = 0; i < iteration; i++) {
    invSqrtInfNorm(dynamics, cost, scalingVectors, N, limitScaling, D, E);
    scaleDataOneStepInPlace(D, E, N, dynamics, cost, scalingVectors);

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

    scalar_t c_temp = std::max(sumOfInfNormOfH / getNumDecisionVariables(), infNormOfh);
    if (c_temp > 1e+4) c_temp = 1e+4;
    if (c_temp < 1e-4) c_temp = 1.0;

    scalar_t gamma = 1.0 / c_temp;

    for (int k = 0; k <= N; ++k) {
      cost[k].dfdxx *= gamma;
      cost[k].dfduu *= gamma;
      cost[k].dfdux *= gamma;
      cost[k].dfdx *= gamma;
      cost[k].dfdu *= gamma;
    }

    cOut *= gamma;

    // /**
    //  * Test start
    //  */
    // auto isEqual = [](scalar_t a, scalar_t b) -> bool {
    //   return std::abs(a - b) < std::numeric_limits<scalar_t>::epsilon() * std::min(std::abs(a), std::abs(b));
    // };
    // vector_t DTest, ETest;
    // vector_t infNormOfHCols = matrixInfNormCols(HTest);
    // vector_t infNormOfGCols = matrixInfNormCols(GTest);

    // DTest = infNormOfHCols.array().max(infNormOfGCols.array());
    // ETest = matrixInfNormRows(GTest);

    // limitScaling(DTest);
    // limitScaling(ETest);

    // std::cerr << "DTest: " << DTest.transpose() << "\n";
    // std::cerr << "ETest   : " << ETest.transpose() << "\n";
    // std::cerr << "G   :\n " << GTest.toDense() << "\n";

    // DTest = DTest.array().sqrt().inverse();
    // ETest = ETest.array().sqrt().inverse();

    // scaleMatrixInPlace(DTest, DTest, HTest);

    // scaleMatrixInPlace(ETest, DTest, GTest);

    // hTest.array() *= DTest.array();

    // scalar_t infNormOfhTest = hTest.lpNorm<Eigen::Infinity>();
    // if (infNormOfhTest < 1e-4) infNormOfhTest = 1.0;

    // const vector_t infNormOfHColsUpdated = matrixInfNormCols(HTest);
    // scalar_t c_tempTest = std::max(infNormOfHColsUpdated.mean(), infNormOfhTest);
    // if (c_tempTest > 1e+4) c_tempTest = 1e+4;
    // if (c_tempTest < 1e-4) c_tempTest = 1.0;

    // scalar_t gammaTest = 1.0 / c_tempTest;

    // HTest *= gamma;
    // hTest *= gamma;

    // ocs2::vector_t DStacked(H.cols()), EStacked(G.rows());
    // int curRow = 0;
    // for (auto& v : D) {
    //   DStacked.segment(curRow, v.size()) = v;
    //   curRow += v.size();
    // }
    // curRow = 0;
    // for (auto& v : E) {
    //   EStacked.segment(curRow, v.size()) = v;
    //   curRow += v.size();
    // }
    // if (!DStacked.isApprox(DTest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "DStacked: " << DStacked.transpose() << "\n";
    //   std::cerr << "DTest   : " << DTest.transpose() << "\n";
    //   std::cerr << "diff    : " << (DStacked - DTest).transpose() << std::endl;
    //   std::cerr << "R: \n" << cost[0].dfduu << std::endl;
    //   std::cerr << "B: \n" << dynamics[0].dfdu << std::endl;
    // }
    // if (!EStacked.isApprox(ETest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "EStacked: " << EStacked.transpose() << "\n";
    //   std::cerr << "ETest: " << ETest.transpose() << "\n";
    //   std::cerr << "diff: " << (ETest - EStacked).transpose() << std::endl;
    // }
    // if (!isEqual(infNormOfhTest, infNormOfh)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "infNormOfhTest: " << infNormOfhTest << "\n";
    //   std::cerr << "infNormOfh: " << infNormOfh << "\n";
    //   std::cerr << "diff: " << (infNormOfhTest - infNormOfh) << std::endl;
    // }
    // if (!isEqual(infNormOfHColsUpdated.sum(), sumOfInfNormOfH)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "infNormOfHColsUpdated.sum(): " << infNormOfHColsUpdated.sum() << "\n";
    //   std::cerr << "sumOfInfNormOfH: " << sumOfInfNormOfH << "\n";
    //   std::cerr << "diff: " << (infNormOfHColsUpdated.sum() - sumOfInfNormOfH) << std::endl;
    // }
    // if (!isEqual(c_temp, c_tempTest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "c_temp: " << c_temp << "\n";
    //   std::cerr << "c_tempTest: " << c_tempTest << "\n";
    //   std::cerr << "diff: " << (c_temp - c_tempTest) << std::endl;
    // }
    // /**
    //  * Test end
    //  */
  }
}

vector_t Pipg::HAbsRowSumInParallel(const std::vector<ScalarFunctionQuadraticApproximation>& cost) const {
  const int N = ocpSize_.numStages;
  const int nu_0 = ocpSize_.numInputs[0];
  vector_t res(getNumDecisionVariables());
  res.head(nu_0) = cost[0].dfduu.cwiseAbs().rowwise().sum();

  int curRow = nu_0;
  for (int k = 1; k < N; k++) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];
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
  const int nx_N = ocpSize_.numStates[N];
  res.tail(nx_N) = cost[N].dfdxx.cwiseAbs().rowwise().sum();
  return res;
}

void Pipg::scaleDataInPlace(const vector_t& D, const vector_t& E, const scalar_t c,
                            std::vector<VectorFunctionLinearApproximation>& dynamics,
                            std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors) const {
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }

  scalingVectors.resize(N);

  /**
   * Scale H & h
   *
   */
  const int nu_0 = ocpSize_.numInputs.front();
  // Scale row - Pre multiply D
  cost.front().dfduu.array().colwise() *= c * D.head(nu_0).array();
  // Scale col - Post multiply D
  cost.front().dfduu *= D.head(nu_0).asDiagonal();

  cost.front().dfdu.array() *= c * D.head(nu_0).array();

  cost.front().dfdux.array().colwise() *= c * D.head(nu_0).array();

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];
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

  const int nx_N = ocpSize_.numStates[N];
  cost[N].dfdxx.array().colwise() *= c * D.tail(nx_N).array();
  cost[N].dfdxx *= D.tail(nx_N).asDiagonal();
  cost[N].dfdx.array() *= c * D.tail(nx_N).array();

  // Scale G & g
  auto& B_0 = dynamics[0].dfdu;
  auto& C_0 = scalingVectors[0];
  const int nx_1 = ocpSize_.numStates[1];
  /**
   * \tilde{B} = E * B * D
   * scaling = E * I * D
   * \tilde{g} = E * g
   */
  B_0.array().colwise() *= E.head(nx_1).array();
  B_0 *= D.head(nu_0).asDiagonal();

  C_0 = E.head(nx_1).array() * D.segment(nu_0, nx_1).array();

  dynamics[0].dfdx.array().colwise() *= E.head(nx_1).array();
  dynamics[0].f.array() *= E.head(nx_1).array();

  currRow = nx_1;
  int currCol = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nu_k = ocpSize_.numInputs[k];
    const int nx_k = ocpSize_.numStates[k];
    const int nx_next = ocpSize_.numStates[k + 1];
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

void Pipg::resize(const OcpSize& ocpSize) {
  if (ocpSize_ == ocpSize) {
    return;
  }

  ocpSize_ = ocpSize;
  const int N = ocpSize_.numStages;

  startIndexArray_.resize(N);
  startIndexArray_.front() = 0;
  std::partial_sum(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end() - 1, startIndexArray_.begin() + 1);

  numDecisionVariables = std::accumulate(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end(), 0);
  numDecisionVariables += std::accumulate(ocpSize_.numInputs.begin(), ocpSize_.numInputs.end(), 0);
  numDynamicsConstraints = std::accumulate(ocpSize_.numStates.begin() + 1, ocpSize_.numStates.end(), 0);

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

void Pipg::getConstraintMatrix(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                               const std::vector<VectorFunctionLinearApproximation>* constraints, const vector_array_t* scalingVectorsPtr,
                               VectorFunctionLinearApproximation& res) const {
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[PIPG] The size of scalingVectors doesn't match the number of stage.");
  }

  // Preallocate full constraint matrix
  auto& G = res.dfdx;
  auto& g = res.f;
  if (constraints != nullptr) {
    G.setZero(getNumDynamicsConstraints() + getNumGeneralEqualityConstraints(), getNumDecisionVariables());

  } else {
    G.setZero(getNumDynamicsConstraints(), getNumDecisionVariables());
  }
  g.setZero(G.rows());

  // Initial state constraint
  const int nu_0 = ocpSize_.numInputs.front();
  const int nx_1 = ocpSize_.numStates[1];
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
    const int nu_k = ocpSize_.numInputs[k];
    const int nx_k = ocpSize_.numStates[k];
    const int nx_next = ocpSize_.numStates[k + 1];

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
  }

  if (constraints != nullptr) {
    currCol = nu_0;
    // === Constraints ===
    // for ocs2 --> C*dx + D*du + e = 0
    // for pipg --> C*dx + D*du = -e
    // Initial general constraints
    const int nc_0 = ocpSize_.numIneqConstraints.front();
    if (nc_0 > 0) {
      const auto& constraint_0 = (*constraints).front();
      G.block(currRow, 0, nc_0, nu_0) = constraint_0.dfdu;
      g.segment(currRow, nc_0) = -constraint_0.f;
      g.segment(currRow, nc_0) -= constraint_0.dfdx * x0;
      currRow += nc_0;
    }

    for (int k = 1; k < N; ++k) {
      const int nc_k = ocpSize_.numIneqConstraints[k];
      const int nu_k = ocpSize_.numInputs[k];
      const int nx_k = ocpSize_.numStates[k];
      if (nc_k > 0) {
        const auto& constraints_k = (*constraints)[k];

        // Add [C, D, 0]
        G.block(currRow, currCol, nc_k, nx_k + nu_k) << constraints_k.dfdx, constraints_k.dfdu;
        // Add [-e]
        g.segment(currRow, nc_k) = -constraints_k.f;
        currRow += nc_k;
      }

      currCol += nx_k + nu_k;
    }

    // Final general constraint
    const int nc_N = ocpSize_.numIneqConstraints[N];
    if (nc_N > 0) {
      const auto& constraints_N = (*constraints)[N];
      G.bottomRightCorner(nc_N, constraints_N.dfdx.cols()) = constraints_N.dfdx;
      g.tail(nc_N) = -constraints_N.f;
    }
  }
}

void Pipg::getConstraintMatrixSparse(const vector_t& x0, const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                     const std::vector<VectorFunctionLinearApproximation>* constraints,
                                     const vector_array_t* scalingVectorsPtr, Eigen::SparseMatrix<scalar_t>& G, vector_t& g) const {
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[PIPG] The size of scalingVectors doesn't match the number of stage.");
  }

  const int nu_0 = ocpSize_.numInputs[0];
  const int nx_1 = ocpSize_.numStates[1];
  const int nx_N = ocpSize_.numStates[N];

  int nnz = nx_1 * (nu_0 + nx_1);
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];
    const int nx_next = ocpSize_.numStates[k + 1];
    nnz += nx_next * (nx_k + nu_k + nx_next);
  }

  if (constraints != nullptr) {
    const int nc_0 = ocpSize_.numIneqConstraints[0];
    nnz += nc_0 * nu_0;
    for (int k = 1; k < N; ++k) {
      const int nx_k = ocpSize_.numStates[k];
      const int nu_k = ocpSize_.numInputs[k];
      const int nc_k = ocpSize_.numIneqConstraints[k];
      nnz += nc_k * (nx_k + nu_k);
    }
    const int nc_N = ocpSize_.numIneqConstraints[N];

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

  if (constraints != nullptr) {
    G.resize(getNumDynamicsConstraints() + getNumGeneralEqualityConstraints(), getNumDecisionVariables());
  } else {
    G.resize(getNumDynamicsConstraints(), getNumDecisionVariables());
  }
  g.setZero(G.rows());

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
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];
    const int nx_next = ocpSize_.numStates[k + 1];

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

  if (constraints != nullptr) {
    currCol = nu_0;
    // === Constraints ===
    // for ocs2 --> C*dx + D*du + e = 0
    // for pipg --> C*dx + D*du = -e
    // Initial general constraints
    const int nc_0 = ocpSize_.numIneqConstraints.front();
    if (nc_0 > 0) {
      const auto& constraint_0 = (*constraints).front();
      emplaceBackMatrix(currRow, 0, constraint_0.dfdu);

      g.segment(currRow, nc_0) = -constraint_0.f;
      g.segment(currRow, nc_0) -= constraint_0.dfdx * x0;
      currRow += nc_0;
    }

    for (int k = 1; k < N; ++k) {
      const int nc_k = ocpSize_.numIneqConstraints[k];
      const int nu_k = ocpSize_.numInputs[k];
      const int nx_k = ocpSize_.numStates[k];
      if (nc_k > 0) {
        const auto& constraints_k = (*constraints)[k];

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
    const int nc_N = ocpSize_.numIneqConstraints[N];
    if (nc_N > 0) {
      const auto& constraints_N = (*constraints)[N];
      emplaceBackMatrix(currRow, currCol, constraints_N.dfdx);
      g.segment(currRow, nc_N) = -constraints_N.f;
    }
  }

  G.setFromTriplets(tripletList.begin(), tripletList.end());
  assert(G.nonZeros() <= nnz);
}

void Pipg::getCostMatrix(const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                         ScalarFunctionQuadraticApproximation& res) const {
  const int N = ocpSize_.numStages;

  // Preallocate full Cost matrices
  auto& H = res.dfdxx;
  auto& h = res.dfdx;
  auto& c = res.f;
  H.setZero(getNumDecisionVariables(), getNumDecisionVariables());
  h.setZero(H.cols());
  c = 0.0;

  // k = 0. Elimination of initial state requires cost adaptation
  vector_t r_0 = cost[0].dfdu;
  r_0 += cost[0].dfdux * x0;

  const int nu_0 = ocpSize_.numInputs[0];
  H.topLeftCorner(nu_0, nu_0) = cost[0].dfduu;
  h.head(nu_0) = r_0;

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];

    // Add [ Q, P'
    //       P, Q ]
    H.block(currRow, currRow, nx_k + nu_k, nx_k + nu_k) << cost[k].dfdxx, cost[k].dfdux.transpose(), cost[k].dfdux, cost[k].dfduu;

    // Add [ q, r]
    h.segment(currRow, nx_k + nu_k) << cost[k].dfdx, cost[k].dfdu;

    // Add nominal cost
    c += cost[k].f;

    currRow += nx_k + nu_k;
  }

  const int nx_N = ocpSize_.numStates[N];
  H.bottomRightCorner(nx_N, nx_N) = cost[N].dfdxx;
  h.tail(nx_N) = cost[N].dfdx;
  c += cost[N].f;
}

void Pipg::getCostMatrixSparse(const vector_t& x0, const std::vector<ScalarFunctionQuadraticApproximation>& cost,
                               Eigen::SparseMatrix<scalar_t>& H, vector_t& h) const {
  const int N = ocpSize_.numStages;

  const int nu_0 = ocpSize_.numInputs[0];
  int nnz = nu_0 * nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];
    nnz += (nx_k + nu_k) * (nx_k + nu_k);
  }
  const int nx_N = ocpSize_.numStates[N];

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

  h.setZero(getNumDecisionVariables());

  // k = 0. Elimination of initial state requires cost adaptation
  vector_t r_0 = cost[0].dfdu;
  r_0 += cost[0].dfdux * x0;

  // R0
  emplaceBackMatrix(0, 0, cost[0].dfduu);
  h.head(nu_0) = r_0;

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize_.numStates[k];
    const int nu_k = ocpSize_.numInputs[k];

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

  H.resize(getNumDecisionVariables(), getNumDecisionVariables());
  H.setFromTriplets(tripletList.begin(), tripletList.end());
  assert(H.nonZeros() <= nnz);
}

void Pipg::GGTAbsRowSumInParallel(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                                  const std::vector<VectorFunctionLinearApproximation>* constraints,
                                  const vector_array_t* scalingVectorsPtr, vector_t& res) {
  const int N = ocpSize_.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }
  if (scalingVectorsPtr != nullptr && scalingVectorsPtr->size() != N) {
    throw std::runtime_error("[PIPG] The size of scalingVectors doesn't match the number of stage.");
  }
  Eigen::setNbThreads(1);  // No multithreading within Eigen.

  matrix_array_t tempMatrixArray(N);
  vector_array_t absRowSumArray(N);

  std::atomic_int timeIndex{0};
  auto task = [&](int workerId) {
    int k;
    while ((k = timeIndex++) < N) {
      const auto nx_next = ocpSize_.numStates[k + 1];
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
  runParallel(std::move(task));

  res.setZero(getNumDynamicsConstraints());
  int curRow = 0;
  for (const auto& v : absRowSumArray) {
    res.segment(curRow, v.size()) = v;
    curRow += v.size();
  }

  Eigen::setNbThreads(0);  // Restore default setup.

  // /**
  //  * *********************************
  //  * ************ Test begin *********
  //  * *********************************
  //  */
  // matrix_t m(getNumDynamicsConstraints(), getNumDynamicsConstraints());
  // const int nx_1 = ocpSize_.numStates[1];
  // const int nx_2 = ocpSize_.numStates[2];
  // const auto& A1 = dynamics[1].dfdx;
  // m.topLeftCorner(nx_1, nx_1 + nx_2) << tempMatrixArray[0],
  //     -(A1 * (scalingVectorsPtr == nullptr ? matrix_t::Identity(nx_1, nx_1) : (*scalingVectorsPtr)[0].asDiagonal().toDenseMatrix()))
  //          .transpose();

  // curRow = nx_1;
  // for (int i = 1; i < N - 1; i++) {
  //   const int nx_i1 = ocpSize_.numStates[i + 1];
  //   const int nx_i2 = ocpSize_.numStates[i + 2];
  //   const auto& ANext = dynamics[i + 1].dfdx;

  //   m.block(curRow, curRow, nx_i1, nx_i1 + nx_i2) << tempMatrixArray[i],
  //       -(ANext * (scalingVectorsPtr == nullptr ? matrix_t::Identity(nx_i1, nx_i1) :
  //       (*scalingVectorsPtr)[i].asDiagonal().toDenseMatrix()))
  //            .transpose();
  //   curRow += nx_i1;
  // }
  // const int nx_N = ocpSize_.numStates[N];
  // m.block(curRow, curRow, nx_N, nx_N) = tempMatrixArray[N - 1];
  // std::cerr << "GGT: \n" << m.selfadjointView<Eigen::Upper>().toDenseMatrix() << "\n\n";
  // /**
  //  * *********************************
  //  * ************ Test end *********
  //  * *********************************
  //  */
}

std::string Pipg::getBenchmarkingInformationDense() const {
  const auto step1v = step1v_.getTotalInMilliseconds();
  const auto step2z = step2z_.getTotalInMilliseconds();
  const auto step3w = step3w_.getTotalInMilliseconds();
  const auto step4CheckConvergence = step4CheckConvergence_.getTotalInMilliseconds();

  const auto benchmarkTotal = step1v + step2z + step3w + step4CheckConvergence;

  std::stringstream infoStream;
  if (benchmarkTotal > 0.0) {
    const scalar_t inPercent = 100.0;
    infoStream << "\n########################################################################\n";
    infoStream << "The benchmarking is computed over " << step1v_.getNumTimedIntervals() << " iterations. \n";
    infoStream << "PIPG Dense Benchmarking\t     :\tAverage time [ms]   (% of total runtime)\n";
    infoStream << "\tstep1v                  :\t" << step1v_.getAverageInMilliseconds() << " [ms] \t\t("
               << step1v / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tstep2z                  :\t" << step2z_.getAverageInMilliseconds() << " [ms] \t\t("
               << step2z / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tstep3w                  :\t" << step3w_.getAverageInMilliseconds() << " [ms] \t\t("
               << step3w / benchmarkTotal * inPercent << "%)\n";
    infoStream << "\tstep4CheckConvergence   :\t" << step4CheckConvergence_.getAverageInMilliseconds() << " [ms] \t\t("
               << step4CheckConvergence / benchmarkTotal * inPercent << "%)\n";
  }
  return infoStream.str();
}

Pipg::~Pipg() {
  if (settings().displayShortSummary) {
    std::cerr << getBenchmarkingInformationDense() << std::endl;
  }
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

vector_t matrixRowwiseAbsSum(const Eigen::SparseMatrix<scalar_t>& mat) {
  vector_t rowAbsSum;
  rowAbsSum.setZero(mat.rows());

  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      int i = it.row();
      rowAbsSum(i) += std::abs(it.value());
    }
  }
  return rowAbsSum;
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

}  // namespace ocs2
