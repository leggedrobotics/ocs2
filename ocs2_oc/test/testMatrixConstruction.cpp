#include <gtest/gtest.h>

#include "ocs2_oc/oc_problem/OcpMatrixConstruction.h"

#include "ocs2_oc/oc_problem/OcpSize.h"
#include "ocs2_oc/test/testProblemsGeneration.h"

class MatrixConstructionTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 5;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);

  MatrixConstructionTest() {
    srand(0);

    x0 = ocs2::vector_t::Random(nx_);
    for (int i = 0; i < N_; i++) {
      dynamicsArray.push_back(ocs2::getRandomDynamics(nx_, nu_));
      costArray.push_back(ocs2::getRandomCost(nx_, nu_));
      constraintsArray.push_back(ocs2::getRandomConstraints(nx_, nu_, nc_));
    }
    costArray.push_back(ocs2::getRandomCost(nx_, nu_));
    constraintsArray.push_back(ocs2::getRandomConstraints(nx_, nu_, nc_));

    ocpSize_ = ocs2::extractSizesFromProblem(dynamicsArray, costArray, &constraintsArray);
  }

  ocs2::OcpSize ocpSize_;
  ocs2::vector_t x0;
  std::vector<ocs2::VectorFunctionLinearApproximation> dynamicsArray;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> costArray;
  std::vector<ocs2::VectorFunctionLinearApproximation> constraintsArray;
};

TEST_F(MatrixConstructionTest, sparseConstraintsApproximation) {
  Eigen::SparseMatrix<ocs2::scalar_t> G;
  ocs2::vector_t g;
  ocs2::vector_array_t scalingVectors(N_);
  for (auto& v : scalingVectors) {
    v = ocs2::vector_t::Random(nx_);
  }
  ocs2::VectorFunctionLinearApproximation constraintsApproximation;
  ocs2::getConstraintMatrix(ocpSize_, x0, dynamicsArray, &constraintsArray, &scalingVectors, constraintsApproximation);
  ocs2::getConstraintMatrixSparse(ocpSize_, x0, dynamicsArray, &constraintsArray, &scalingVectors, G, g);

  EXPECT_TRUE(constraintsApproximation.dfdx.isApprox(G.toDense()));
  EXPECT_TRUE(constraintsApproximation.f.isApprox(g));
}

TEST_F(MatrixConstructionTest, sparseCostApproximation) {
  Eigen::SparseMatrix<ocs2::scalar_t> H;
  ocs2::vector_t h;

  ocs2::ScalarFunctionQuadraticApproximation costApproximation;
  ocs2::getCostMatrix(ocpSize_, x0, costArray, costApproximation);
  ocs2::getCostMatrixSparse(ocpSize_, x0, costArray, H, h);

  EXPECT_TRUE(costApproximation.dfdxx.isApprox(H.toDense()));
  EXPECT_TRUE(costApproximation.dfdx.isApprox(h));
}
