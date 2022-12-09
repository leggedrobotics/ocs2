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

#include <gtest/gtest.h>
#include <Eigen/Sparse>

#include <ocs2_oc/oc_problem/OcpToKkt.h>
#include <ocs2_oc/test/testProblemsGeneration.h>
#include <ocs2_qp_solver/QpSolver.h>

#include "ocs2_slp/pipg/PipgSolver.h"
#include "ocs2_slp/pipg/SingleThreadPipg.h"

ocs2::pipg::Settings configurePipg(size_t maxNumIterations, ocs2::scalar_t absoluteTolerance, ocs2::scalar_t relativeTolerance,
                                   bool verbose) {
  ocs2::pipg::Settings settings;
  settings.maxNumIterations = maxNumIterations;
  settings.absoluteTolerance = absoluteTolerance;
  settings.relativeTolerance = relativeTolerance;
  settings.checkTerminationInterval = 1;
  settings.displayShortSummary = verbose;

  return settings;
}

class PIPGSolverTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 0;
  static constexpr size_t numDecisionVariables_ = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints_ = N_ * (nx_ + nc_);
  static constexpr size_t numThreads_ = 8;
  static constexpr bool verbose_ = true;

  PIPGSolverTest() : solver(configurePipg(30000, 1e-10, 1e-3, verbose_)) {
    srand(10);

    // Construct OCP problem
    x0 = ocs2::vector_t::Random(nx_);

    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(ocs2::getRandomDynamics(nx_, nu_));
      costArray.push_back(ocs2::getRandomCost(nx_, nu_));
      constraintsArray.push_back(ocs2::getRandomConstraints(nx_, nu_, nc_));
    }
    costArray.push_back(ocs2::getRandomCost(nx_, nu_));
    constraintsArray.push_back(ocs2::getRandomConstraints(nx_, nu_, nc_));

    solver.resize(ocs2::extractSizesFromProblem(dynamicsArray, costArray, &constraintsArray));
    ocs2::getCostMatrix(solver.size(), x0, costArray, costApproximation);
    ocs2::getConstraintMatrix(solver.size(), x0, dynamicsArray, nullptr, nullptr, constraintsApproximation);
  }

  ocs2::vector_t x0;
  ocs2::ScalarFunctionQuadraticApproximation costApproximation;
  ocs2::VectorFunctionLinearApproximation constraintsApproximation;
  std::vector<ocs2::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<ocs2::VectorFunctionLinearApproximation> constraintsArray;

  ocs2::PipgSolver solver;
  ocs2::ThreadPool threadPool{numThreads_ - 1u, 50};
};

constexpr size_t PIPGSolverTest::N_;
constexpr size_t PIPGSolverTest::nx_;
constexpr size_t PIPGSolverTest::nu_;
constexpr size_t PIPGSolverTest::nc_;
constexpr size_t PIPGSolverTest::numDecisionVariables_;
constexpr size_t PIPGSolverTest::numConstraints_;
constexpr size_t PIPGSolverTest::numThreads_;
constexpr bool PIPGSolverTest::verbose_;

TEST_F(PIPGSolverTest, correctness) {
  // ocs2::qp_solver::SolveDenseQP use  Gz + g = 0 for constraints
  auto QPconstraints = constraintsApproximation;
  QPconstraints.f = -QPconstraints.f;
  ocs2::vector_t primalSolutionQP;
  std::tie(primalSolutionQP, std::ignore) = ocs2::qp_solver::solveDenseQp(costApproximation, QPconstraints);

  Eigen::JacobiSVD<ocs2::matrix_t> svd(costApproximation.dfdxx);
  ocs2::vector_t s = svd.singularValues();
  const ocs2::scalar_t lambda = s(0);
  const ocs2::scalar_t mu = s(svd.rank() - 1);
  Eigen::JacobiSVD<ocs2::matrix_t> svdGTG(constraintsApproximation.dfdx.transpose() * constraintsApproximation.dfdx);
  const ocs2::scalar_t sigma = svdGTG.singularValues()(0);
  const ocs2::pipg::PipgBounds pipgBounds{mu, lambda, sigma};

  ocs2::vector_t primalSolutionPIPG;
  std::ignore = ocs2::pipg::singleThreadPipg(solver.settings(), costApproximation.dfdxx.sparseView(), costApproximation.dfdx,
                                             constraintsApproximation.dfdx.sparseView(), constraintsApproximation.f,
                                             ocs2::vector_t::Ones(solver.getNumDynamicsConstraints()), pipgBounds, primalSolutionPIPG);

  ocs2::vector_array_t scalingVectors(N_, ocs2::vector_t::Ones(nx_));
  ocs2::vector_array_t X, U;
  std::ignore = solver.solve(threadPool, x0, dynamicsArray, costArray, nullptr, scalingVectors, nullptr, pipgBounds, X, U);

  ocs2::vector_t primalSolutionPIPGParallel;
  ocs2::toKktSolution(X, U, primalSolutionPIPGParallel);

  auto calculateConstraintViolation = [&](const ocs2::vector_t& sol) -> ocs2::scalar_t {
    return (constraintsApproximation.dfdx * sol - constraintsApproximation.f).cwiseAbs().maxCoeff();
  };
  auto calculateCost = [&](const ocs2::vector_t& sol) -> ocs2::scalar_t {
    return (0.5 * sol.transpose() * costApproximation.dfdxx * sol + costApproximation.dfdx.transpose() * sol)(0);
  };
  ocs2::scalar_t QPCost = calculateCost(primalSolutionQP);
  ocs2::scalar_t QPConstraintViolation = calculateConstraintViolation(primalSolutionQP);

  ocs2::scalar_t PIPGCost = calculateCost(primalSolutionPIPG);
  ocs2::scalar_t PIPGConstraintViolation = calculateConstraintViolation(primalSolutionPIPG);

  ocs2::scalar_t PIPGParallelCost = calculateCost(primalSolutionPIPGParallel);
  ocs2::scalar_t PIPGParallelCConstraintViolation = calculateConstraintViolation(primalSolutionPIPGParallel);

  if (verbose_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ [TestPIPG] Correctness ++++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";

    std::cerr << "mu:  " << mu << " lambda: " << lambda << " sigma: " << sigma << "\n";

    std::cerr << "QP-PIPG:  " << (primalSolutionQP - primalSolutionPIPG).cwiseAbs().sum() << "\n";
    std::cerr << "PIPG-PIPGParallel:  " << (primalSolutionPIPG - primalSolutionPIPGParallel).cwiseAbs().sum() << "\n";

    std::cerr << "QP:\n";
    std::cerr << "cost:   " << QPCost << "    "
              << "constraint-violation:   " << QPConstraintViolation << "\n\n";

    std::cerr << "PIPG:\n";
    std::cerr << "cost:   " << PIPGCost << "    "
              << "constraint-violation:   " << PIPGConstraintViolation << "\n\n";

    std::cerr << "PIPGParallel:\n";
    std::cerr << "cost:   " << PIPGParallelCost << "    "
              << "constraint-violation:   " << PIPGParallelCConstraintViolation << "\n\n"
              << std::endl;
  }
  EXPECT_TRUE(primalSolutionQP.isApprox(primalSolutionPIPG, solver.settings().absoluteTolerance * 10.0))
      << "Inf-norm of (QP - PIPG): " << (primalSolutionQP - primalSolutionPIPG).cwiseAbs().maxCoeff();

  EXPECT_TRUE(primalSolutionPIPGParallel.isApprox(primalSolutionPIPG, solver.settings().absoluteTolerance * 10.0))
      << "Inf-norm of (PIPG - PIPGParallel): " << (primalSolutionPIPGParallel - primalSolutionPIPG).cwiseAbs().maxCoeff();

  // Threshold is the (absoluteTolerance) * (2-Norm of the hessian H)[lambda]
  EXPECT_TRUE(std::abs(QPCost - PIPGCost) < solver.settings().absoluteTolerance * lambda)
      << "Absolute diff is [" << std::abs(QPCost - PIPGCost) << "] which is larger than [" << solver.settings().absoluteTolerance * lambda
      << "]";

  EXPECT_TRUE(std::abs(PIPGParallelCost - PIPGCost) < solver.settings().absoluteTolerance)
      << "Absolute diff is [" << std::abs(PIPGParallelCost - PIPGCost) << "] which is larger than ["
      << solver.settings().absoluteTolerance * lambda << "]";

  ASSERT_TRUE(std::abs(PIPGConstraintViolation) < solver.settings().absoluteTolerance);
  EXPECT_TRUE(std::abs(QPConstraintViolation - PIPGConstraintViolation) < solver.settings().absoluteTolerance * 10.0);
  EXPECT_TRUE(std::abs(PIPGParallelCConstraintViolation - PIPGConstraintViolation) < solver.settings().absoluteTolerance * 10.0);
}