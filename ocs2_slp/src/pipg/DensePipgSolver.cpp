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

#include "ocs2_slp/pipg/DensePipgSolver.h"

#include <iostream>
#include <numeric>

namespace ocs2 {
namespace pipg {

SolverStatus densePipg(const pipg::Settings& settings, const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h,
                       const Eigen::SparseMatrix<scalar_t>& G, const vector_t& g, const vector_t& EInv, const scalar_t mu,
                       const scalar_t lambda, const scalar_t sigma, vector_t& stackedSolution, DensePipgBenchmark* timerPtr) {
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
  while (k < settings.maxNumIterations && !isConverged) {
    const scalar_t alpha = 2.0 / ((k + 1.0) * mu + 2.0 * lambda);
    const scalar_t beta = (k + 1) * mu / (2.0 * sigma);

    z_old.swap(z);
    if (timerPtr != nullptr) {
      timerPtr->vComputation.startTimer();
    }
    // v = w + beta * (G * z - g);
    v = -g;
    v.noalias() += G * z;
    v *= beta;
    v += w;
    if (timerPtr != nullptr) {
      timerPtr->vComputation.endTimer();
    }

    if (timerPtr != nullptr) {
      timerPtr->zComputation.startTimer();
    }
    // z = z_old - alpha * (H * z_old + h + G.transpose() * v);
    z = -h;
    z.noalias() -= (H * z_old);
    z.noalias() -= (G.transpose() * v);
    z *= alpha;
    z.noalias() += z_old;
    if (timerPtr != nullptr) {
      timerPtr->zComputation.endTimer();
    }

    if (timerPtr != nullptr) {
      timerPtr->wComputation.startTimer();
    }
    // w = w + beta * (G * z - g);
    w -= beta * g;
    w.noalias() += beta * (G * z);
    if (timerPtr != nullptr) {
      timerPtr->wComputation.endTimer();
    }

    if (timerPtr != nullptr) {
      timerPtr->convergenceCheck.startTimer();
    }
    if (k % settings.checkTerminationInterval == 0) {
      const scalar_t zNorm = z.squaredNorm();

      constraintsViolation.noalias() = G * z;
      constraintsViolation -= g;
      constraintsViolation.cwiseProduct(EInv);
      constraintsViolationInfNorm = constraintsViolation.lpNorm<Eigen::Infinity>();

      const vector_t z_delta = z - z_old;
      const scalar_t z_deltaNorm = z_delta.squaredNorm();
      isConverged =
          constraintsViolationInfNorm <= settings.absoluteTolerance &&
          (z_deltaNorm <= settings.relativeTolerance * settings.relativeTolerance * zNorm || z_deltaNorm <= settings.absoluteTolerance);
    }
    if (timerPtr != nullptr) {
      timerPtr->convergenceCheck.endTimer();
    }

    ++k;
  }  // end of while loop

  stackedSolution.swap(z);
  pipg::SolverStatus status = isConverged ? pipg::SolverStatus::SUCCESS : pipg::SolverStatus::MAX_ITER;

  if (settings.displayShortSummary) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ Dense PIPG ++++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++\n";
    std::cerr << "Solver status: " << pipg::toString(status) << "\n";
    std::cerr << "Number of Iterations: " << k << " out of " << settings.maxNumIterations << "\n";
    std::cerr << "Norm of delta primal solution: " << (stackedSolution - z_old).norm() << "\n";
    std::cerr << "Constraints violation : " << constraintsViolationInfNorm << "\n";
    if (timerPtr != nullptr) {
      const auto step1v = timerPtr->vComputation.getTotalInMilliseconds();
      const auto step2z = timerPtr->zComputation.getTotalInMilliseconds();
      const auto step3w = timerPtr->wComputation.getTotalInMilliseconds();
      const auto step4CheckConvergence = timerPtr->convergenceCheck.getTotalInMilliseconds();
      const auto benchmarkTotal = step1v + step2z + step3w + step4CheckConvergence;
      std::cerr << "\n########################################################################\n";
      std::cerr << "The benchmarking is computed over " << k << " iterations. \n";
      std::cerr << "PIPG Dense Benchmarking\t     :\tAverage time [ms]   (% of total runtime)\n";
      std::cerr << "\tvComputation         :\t" << timerPtr->vComputation.getAverageInMilliseconds() << " [ms] \t\t("
                << step1v / benchmarkTotal * 100.0 << "%)\n";
      std::cerr << "\tzComputation         :\t" << timerPtr->zComputation.getAverageInMilliseconds() << " [ms] \t\t("
                << step2z / benchmarkTotal * 100.0 << "%)\n";
      std::cerr << "\twComputation         :\t" << timerPtr->wComputation.getAverageInMilliseconds() << " [ms] \t\t("
                << step3w / benchmarkTotal * 100.0 << "%)\n";
      std::cerr << "\tCheckConvergence     :\t" << timerPtr->convergenceCheck.getAverageInMilliseconds() << " [ms] \t\t("
                << step4CheckConvergence / benchmarkTotal * 100.0 << "%)\n";
    }
  }

  return status;
}

void unpackSolution(const OcpSize& ocpSize, const vector_t& stackedSolution, const vector_t x0, vector_array_t& xTrajectory,
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

void packSolution(const vector_array_t& xTrajectory, const vector_array_t& uTrajectory, vector_t& stackedSolution) {
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

}  // namespace pipg
}  // namespace ocs2
