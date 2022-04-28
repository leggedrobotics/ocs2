#include <gtest/gtest.h>

#include "ocs2_pipg/HelperFunctions.h"

#include <ocs2_oc/oc_problem/OcpMatrixConstruction.h>
#include <ocs2_oc/test/testProblemsGeneration.h>

class HelperFunctionTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 0;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);
  static constexpr bool verbose = true;

  HelperFunctionTest() : threadPool_(5, 99) {
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
    ocpSize_ = ocs2::extractSizesFromProblem(dynamicsArray, costArray, nullptr);

    ocs2::getCostMatrix(ocpSize_, x0, costArray, costApproximation);
  }

  ocs2::OcpSize ocpSize_;
  ocs2::vector_t x0;
  ocs2::ScalarFunctionQuadraticApproximation costApproximation;

  std::vector<ocs2::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<ocs2::VectorFunctionLinearApproximation> constraintsArray;

  ocs2::ThreadPool threadPool_;
};

TEST_F(HelperFunctionTest, hessianAbsRowSum) {
  ocs2::vector_t rowwiseSum = ocs2::pipg::hessianAbsRowSum(ocpSize_, costArray);
  EXPECT_TRUE(rowwiseSum.isApprox(costApproximation.dfdxx.cwiseAbs().rowwise().sum())) << "rowSum:\n" << rowwiseSum.transpose();
}

TEST_F(HelperFunctionTest, GGTAbsRowSumInParallel) {
  ocs2::VectorFunctionLinearApproximation constraintsApproximation;
  ocs2::vector_array_t scalingVectors(N_);
  for (auto& v : scalingVectors) {
    v = ocs2::vector_t::Random(nx_);
  }

  ocs2::getConstraintMatrix(ocpSize_, x0, dynamicsArray, nullptr, &scalingVectors, constraintsApproximation);
  ocs2::vector_t rowwiseSum = ocs2::pipg::GGTAbsRowSumInParallel(ocpSize_, dynamicsArray, nullptr, &scalingVectors, threadPool_);
  ocs2::matrix_t GGT = constraintsApproximation.dfdx * constraintsApproximation.dfdx.transpose();
  EXPECT_TRUE(rowwiseSum.isApprox(GGT.cwiseAbs().rowwise().sum()));
}