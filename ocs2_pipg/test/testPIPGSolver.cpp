#include <gtest/gtest.h>

#include "ocs2_pipg/PIPG.h"

#include <ocs2_qp_solver/QpSolver.h>
#include <ocs2_qp_solver/test/testProblemsGeneration.h>

#include <Eigen/Sparse>

class PIPGSolverTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 5;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 0;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);
  static constexpr bool verbose = true;

  PIPGSolverTest()
      : pipgSolver({
            8,        // nThreads
            50,       // threadPriority
            30000,    // maxNumIterations
            1e-10,    // absoluteTolerance
            1e-3,     // relativeTolerance
            1,        // checkTerminationInterval
            verbose,  // displayShortSummary
        }) {
    srand(10);

    // Construct OCP problem
    lqProblem = ocs2::qp_solver::generateRandomLqProblem(N_, nx_, nu_, nc_);
    x0 = ocs2::vector_t::Random(nx_);

    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(lqProblem[i].dynamics);
      costArray.push_back(lqProblem[i].cost);
      constraintsArray.push_back(lqProblem[i].constraints);
    }
    costArray.push_back(lqProblem[N_].cost);
    constraintsArray.push_back(lqProblem[N_].constraints);

    pipgSolver.resize(ocs2::pipg::extractSizesFromProblem(dynamicsArray, costArray, &constraintsArray));
    pipgSolver.getCostMatrix(x0, costArray, costApproximation);
    pipgSolver.getConstraintMatrix(x0, dynamicsArray, nullptr, nullptr, constraintsApproximation);
  }

  std::vector<ocs2::qp_solver::LinearQuadraticStage> lqProblem;
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

TEST_F(PIPGSolverTest, PIPGcorrectness) {
  // SolveDenseQP use  Gz + g = 0 for constraints
  auto QPconstraints = constraintsApproximation;
  QPconstraints.f = -QPconstraints.f;
  ocs2::vector_t primalSolutionQP;
  std::tie(primalSolutionQP, std::ignore) = ocs2::qp_solver::solveDenseQp(costApproximation, QPconstraints);
  // primalSolutionQP.setZero(numDecisionVariables);

  Eigen::JacobiSVD<ocs2::matrix_t> svd(costApproximation.dfdxx);
  ocs2::vector_t s = svd.singularValues();
  ocs2::scalar_t lambda = s(0);
  ocs2::scalar_t mu = s(svd.rank() - 1);
  Eigen::JacobiSVD<ocs2::matrix_t> svdGTG(constraintsApproximation.dfdx.transpose() * constraintsApproximation.dfdx);
  ocs2::scalar_t sigma = svdGTG.singularValues()(0);

  ocs2::vector_t primalSolutionPIPG;
  pipgSolver.solveDenseQP(costApproximation, constraintsApproximation, ocs2::vector_t::Ones(pipgSolver.getNumDynamicsConstraints()), mu,
                          lambda, sigma, primalSolutionPIPG);
  // primalSolutionPIPG.setZero(numDecisionVariables);

  ocs2::vector_t primalSolutionPIPGParallel;
  ocs2::vector_array_t scalingVectors(N_, ocs2::vector_t::Ones(nx_));

  pipgSolver.solveOCPInParallel(x0, dynamicsArray, costArray, nullptr, scalingVectors, nullptr, mu, lambda, sigma, costApproximation,
                                constraintsApproximation);
  pipgSolver.getStackedSolution(primalSolutionPIPGParallel);
  // primalSolutionPIPGParallel.setZero(numDecisionVariables);

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

TEST_F(PIPGSolverTest, preConditioning) {
  auto costScaledWithFactors = costApproximation;
  auto constraintScaledWithFactors = constraintsApproximation;
  ocs2::vector_t D, E;
  ocs2::scalar_t c;
  Eigen::SparseMatrix<ocs2::scalar_t> HScaledFromCalculation = costApproximation.dfdxx.sparseView(),
                                      GScaledFromCalculation = constraintsApproximation.dfdx.sparseView();
  ocs2::vector_t hScaledFromCalculation = costApproximation.dfdx;

  pipgSolver.calculatePreConditioningFactors(HScaledFromCalculation, hScaledFromCalculation, GScaledFromCalculation, 5, D, E, c);

  costScaledWithFactors.dfdxx = c * D.asDiagonal() * costScaledWithFactors.dfdxx * D.asDiagonal();
  costScaledWithFactors.dfdx = c * D.asDiagonal() * costScaledWithFactors.dfdx;
  constraintScaledWithFactors.dfdx = E.asDiagonal() * constraintScaledWithFactors.dfdx * D.asDiagonal();
  constraintScaledWithFactors.f = E.asDiagonal() * constraintScaledWithFactors.f;

  // The scaled matrix given by calculatePreConditioningFactors function and the one calculated with D, E and c factors should be the same.
  EXPECT_TRUE(costScaledWithFactors.dfdxx.isApprox(HScaledFromCalculation.toDense()));       // H
  EXPECT_TRUE(costScaledWithFactors.dfdx.isApprox(hScaledFromCalculation));                  // h
  EXPECT_TRUE(constraintScaledWithFactors.dfdx.isApprox(GScaledFromCalculation.toDense()));  // G

  // Normalized the inf-Norm of both rows and cols of KKT matrix to 1
  ocs2::matrix_t KKT(costApproximation.dfdxx.rows() + constraintsApproximation.dfdx.rows(),
                     costApproximation.dfdxx.rows() + constraintsApproximation.dfdx.rows());
  KKT << costApproximation.dfdxx, constraintsApproximation.dfdx.transpose(), constraintsApproximation.dfdx,
      ocs2::matrix_t::Zero(constraintsApproximation.dfdx.rows(), constraintsApproximation.dfdx.rows());

  ocs2::matrix_t KKTScaled(costApproximation.dfdxx.rows() + constraintsApproximation.dfdx.rows(),
                           costApproximation.dfdxx.rows() + constraintsApproximation.dfdx.rows());
  KKTScaled << costScaledWithFactors.dfdxx, constraintScaledWithFactors.dfdx.transpose(), constraintScaledWithFactors.dfdx,
      ocs2::matrix_t::Zero(constraintsApproximation.dfdx.rows(), constraintsApproximation.dfdx.rows());

  ocs2::vector_t infNormOfKKT = KKT.cwiseAbs().rowwise().maxCoeff();
  ocs2::vector_t infNormOfKKTScaled = KKTScaled.cwiseAbs().rowwise().maxCoeff();
  EXPECT_LT((infNormOfKKTScaled.array() - 1).abs().maxCoeff(), (infNormOfKKT.array() - 1).abs().maxCoeff());

  std::vector<ocs2::vector_t> scalingVectors;
  pipgSolver.scaleDataInPlace(D, E, c, dynamicsArray, costArray, scalingVectors);

  ocs2::ScalarFunctionQuadraticApproximation costConstructedFromScaledDataArray;
  ocs2::VectorFunctionLinearApproximation constraintConstructedFromScaledDataArray;
  pipgSolver.getCostMatrix(x0, costArray, costConstructedFromScaledDataArray);
  pipgSolver.getConstraintMatrix(x0, dynamicsArray, nullptr, &scalingVectors, constraintConstructedFromScaledDataArray);

  EXPECT_TRUE(costScaledWithFactors.dfdxx.isApprox(costConstructedFromScaledDataArray.dfdxx));            // H
  EXPECT_TRUE(costScaledWithFactors.dfdx.isApprox(costConstructedFromScaledDataArray.dfdx));              // h
  EXPECT_TRUE(constraintScaledWithFactors.dfdx.isApprox(constraintConstructedFromScaledDataArray.dfdx));  // G
  EXPECT_TRUE(constraintScaledWithFactors.f.isApprox(constraintConstructedFromScaledDataArray.f));        // g
}

TEST_F(PIPGSolverTest, preConditioningSimplified) {
  auto costScaledWithFactors = costApproximation;
  auto constraintScaledWithFactors = constraintsApproximation;
  ocs2::vector_t D, E;
  ocs2::scalar_t c;
  Eigen::SparseMatrix<ocs2::scalar_t> HScaledFromCalculation = costApproximation.dfdxx.sparseView(),
                                      GScaledFromCalculation = constraintsApproximation.dfdx.sparseView();
  ocs2::vector_t hScaledFromCalculation = costApproximation.dfdx;

  pipgSolver.calculatePreConditioningFactors(HScaledFromCalculation, hScaledFromCalculation, GScaledFromCalculation, 5, D, E, c);

  ocs2::vector_array_t DArray, EArray;
  ocs2::scalar_t cMy;
  ocs2::vector_array_t scalingVectors(N_);
  pipgSolver.preConditioningInPlaceInParallel(x0, dynamicsArray, costArray, 5, DArray, EArray, scalingVectors, cMy,
                                              costApproximation.dfdxx.sparseView(), costApproximation.dfdx,
                                              constraintsApproximation.dfdx.sparseView());

  ocs2::vector_t DStacked(pipgSolver.getNumDecisionVariables()), EStacked(pipgSolver.getNumDynamicsConstraints());
  int curRow = 0;
  for (auto& v : DArray) {
    DStacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }
  curRow = 0;
  for (auto& v : EArray) {
    EStacked.segment(curRow, v.size()) = v;
    curRow += v.size();
  }

  // The scaled matrix given by calculatePreConditioningFactors function and the one calculated with D, E and c factors should be the
  // same.
  EXPECT_TRUE(DStacked.isApprox(D));
  EXPECT_TRUE(EStacked.isApprox(E));
  EXPECT_FLOAT_EQ(c, cMy);

  ocs2::ScalarFunctionQuadraticApproximation costConstructedFromScaledDataArray;
  ocs2::VectorFunctionLinearApproximation constraintConstructedFromScaledDataArray;
  ocs2::vector_t gScaledFromCalculation = E.cwiseProduct(constraintsApproximation.f);
  pipgSolver.getCostMatrix(x0, costArray, costConstructedFromScaledDataArray);
  pipgSolver.getConstraintMatrix(x0, dynamicsArray, nullptr, &scalingVectors, constraintConstructedFromScaledDataArray);

  EXPECT_TRUE(HScaledFromCalculation.isApprox(costConstructedFromScaledDataArray.dfdxx));       // H
  EXPECT_TRUE(hScaledFromCalculation.isApprox(costConstructedFromScaledDataArray.dfdx));        // h
  EXPECT_TRUE(GScaledFromCalculation.isApprox(constraintConstructedFromScaledDataArray.dfdx));  // G
  EXPECT_TRUE(gScaledFromCalculation.isApprox(constraintConstructedFromScaledDataArray.f));     // g
}

TEST_F(PIPGSolverTest, constructSparseCostApproximation) {
  Eigen::SparseMatrix<ocs2::scalar_t> H;
  ocs2::vector_t h;
  pipgSolver.getCostMatrixSparse(x0, costArray, H, h);

  EXPECT_TRUE(costApproximation.dfdxx.isApprox(H.toDense()));
  EXPECT_TRUE(costApproximation.dfdx.isApprox(h));
}

TEST_F(PIPGSolverTest, HAbsRowSumInParallel) {
  ocs2::vector_t rowwiseSum = pipgSolver.HAbsRowSumInParallel(costArray);

  EXPECT_TRUE(rowwiseSum.isApprox(costApproximation.dfdxx.cwiseAbs().rowwise().sum()));
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

TEST_F(PIPGSolverTest, constructSparseConstraintsApproximation) {
  Eigen::SparseMatrix<ocs2::scalar_t> G;
  ocs2::vector_t g;
  ocs2::vector_array_t scalingVectors(N_);
  for (auto& v : scalingVectors) {
    v = ocs2::vector_t::Random(nx_);
  }
  ocs2::VectorFunctionLinearApproximation constraintsApproximation;
  pipgSolver.getConstraintMatrix(x0, dynamicsArray, &constraintsArray, &scalingVectors, constraintsApproximation);

  pipgSolver.getConstraintMatrixSparse(x0, dynamicsArray, &constraintsArray, &scalingVectors, G, g);

  EXPECT_TRUE(constraintsApproximation.dfdx.isApprox(G.toDense()));
  EXPECT_TRUE(constraintsApproximation.f.isApprox(g));
}