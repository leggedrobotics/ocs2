#include <gtest/gtest.h>

#include "ocs2_pipg/PIPG.h"

#include <Eigen/Sparse>

#include <ocs2_oc/oc_problem/OcpMatrixConstruction.h>
#include <ocs2_oc/test/testProblemsGeneration.h>
#include <ocs2_qp_solver/QpSolver.h>

ocs2::pipg::Settings configurePipg(size_t nThreads, size_t maxNumIterations, ocs2::scalar_t absoluteTolerance,
                                   ocs2::scalar_t relativeTolerance, bool verbose) {
  ocs2::pipg::Settings settings;
  settings.nThreads = nThreads;
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
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);
  static constexpr bool verbose = true;

  PIPGSolverTest() : pipgSolver(configurePipg(8, 30000, 1e-10, 1e-3, verbose)) {
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

    pipgSolver.resize(ocs2::extractSizesFromProblem(dynamicsArray, costArray, &constraintsArray));
    ocs2::getCostMatrix(pipgSolver.size(), x0, costArray, costApproximation);
    ocs2::getConstraintMatrix(pipgSolver.size(), x0, dynamicsArray, nullptr, nullptr, constraintsApproximation);
  }

  ocs2::vector_t x0;
  ocs2::ScalarFunctionQuadraticApproximation costApproximation;
  ocs2::VectorFunctionLinearApproximation constraintsApproximation;
  std::vector<ocs2::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<ocs2::VectorFunctionLinearApproximation> constraintsArray;

  ocs2::Pipg pipgSolver;
};

constexpr size_t PIPGSolverTest::numDecisionVariables;
constexpr size_t PIPGSolverTest::numConstraints;

TEST_F(PIPGSolverTest, correctness) {
  // ocs2::qp_solver::SolveDenseQP use  Gz + g = 0 for constraints
  auto QPconstraints = constraintsApproximation;
  QPconstraints.f = -QPconstraints.f;
  ocs2::vector_t primalSolutionQP;
  std::tie(primalSolutionQP, std::ignore) = ocs2::qp_solver::solveDenseQp(costApproximation, QPconstraints);

  Eigen::JacobiSVD<ocs2::matrix_t> svd(costApproximation.dfdxx);
  ocs2::vector_t s = svd.singularValues();
  ocs2::scalar_t lambda = s(0);
  ocs2::scalar_t mu = s(svd.rank() - 1);
  Eigen::JacobiSVD<ocs2::matrix_t> svdGTG(constraintsApproximation.dfdx.transpose() * constraintsApproximation.dfdx);
  ocs2::scalar_t sigma = svdGTG.singularValues()(0);

  ocs2::vector_t primalSolutionPIPG;
  pipgSolver.solveDenseQP(costApproximation.dfdxx.sparseView(), costApproximation.dfdx, constraintsApproximation.dfdx.sparseView(),
                          constraintsApproximation.f, ocs2::vector_t::Ones(pipgSolver.getNumDynamicsConstraints()), mu, lambda, sigma,
                          primalSolutionPIPG);

  ocs2::vector_t primalSolutionPIPGParallel;
  ocs2::vector_array_t scalingVectors(N_, ocs2::vector_t::Ones(nx_));

  pipgSolver.solveOCPInParallel(x0, dynamicsArray, costArray, nullptr, scalingVectors, nullptr, mu, lambda, sigma, costApproximation,
                                constraintsApproximation);
  pipgSolver.getStackedSolution(primalSolutionPIPGParallel);

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

  if (verbose) {
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
  EXPECT_TRUE(primalSolutionQP.isApprox(primalSolutionPIPG, pipgSolver.settings().absoluteTolerance * 10.0))
      << "Inf-norm of (QP - PIPG): " << (primalSolutionQP - primalSolutionPIPG).cwiseAbs().maxCoeff();

  EXPECT_TRUE(primalSolutionPIPGParallel.isApprox(primalSolutionPIPG, pipgSolver.settings().absoluteTolerance * 10.0))
      << "Inf-norm of (PIPG - PIPGParallel): " << (primalSolutionPIPGParallel - primalSolutionPIPG).cwiseAbs().maxCoeff();

  // Threshold is the (absoluteTolerance) * (2-Norm of the hessian H)[lambda]
  EXPECT_TRUE(std::abs(QPCost - PIPGCost) < pipgSolver.settings().absoluteTolerance * lambda)
      << "Absolute diff is [" << std::abs(QPCost - PIPGCost) << "] which is larger than ["
      << pipgSolver.settings().absoluteTolerance * lambda << "]";

  EXPECT_TRUE(std::abs(PIPGParallelCost - PIPGCost) < pipgSolver.settings().absoluteTolerance)
      << "Absolute diff is [" << std::abs(PIPGParallelCost - PIPGCost) << "] which is larger than ["
      << pipgSolver.settings().absoluteTolerance * lambda << "]";

  ASSERT_TRUE(std::abs(PIPGConstraintViolation) < pipgSolver.settings().absoluteTolerance);
  EXPECT_TRUE(std::abs(QPConstraintViolation - PIPGConstraintViolation) < pipgSolver.settings().absoluteTolerance * 10.0);
  EXPECT_TRUE(std::abs(PIPGParallelCConstraintViolation - PIPGConstraintViolation) < pipgSolver.settings().absoluteTolerance * 10.0);
}

TEST_F(PIPGSolverTest, descaleSolution) {
  ocs2::vector_array_t D(2 * N_);
  ocs2::vector_t DStacked(numDecisionVariables);
  ocs2::vector_array_t x(N_ + 1), u(N_);
  x[0].setRandom(nx_);
  for (int i = 0; i < N_; i++) {
    D[2 * i].setRandom(nu_);
    D[2 * i + 1].setRandom(nx_);
    u[i].setRandom(nu_);
    x[i + 1].setRandom(nx_);
  }
  int curRow = 0;
  for (auto& v : D) {
    DStacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }
  std::cerr << "DStacked:\n" << DStacked.transpose() << "\n\n";
  ocs2::vector_t packedSolution;
  pipgSolver.packSolution(x, u, packedSolution);
  std::cerr << "packedSolution:\n" << packedSolution.transpose() << "\n\n";

  packedSolution.array() *= DStacked.array();

  ocs2::vector_t packedSolutionMy;
  pipgSolver.descaleSolution(D, x, u);
  pipgSolver.packSolution(x, u, packedSolutionMy);
  EXPECT_TRUE(packedSolutionMy.isApprox(packedSolution)) << std::setprecision(6) << "DescaledSolution: \n"
                                                         << packedSolutionMy.transpose() << "\nIt should be \n"
                                                         << packedSolution.transpose();
}
