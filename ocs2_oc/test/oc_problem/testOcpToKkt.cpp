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

#include "ocs2_oc/oc_problem/OcpSize.h"
#include "ocs2_oc/oc_problem/OcpToKkt.h"

#include "ocs2_oc/test/testProblemsGeneration.h"

class OcpToKktTest : public testing::Test {
 protected:
  // x_0, x_1, ... x_{N - 1}, X_{N}
  static constexpr size_t N_ = 10;  // numStages
  static constexpr size_t nx_ = 4;
  static constexpr size_t nu_ = 3;
  static constexpr size_t nc_ = 5;
  static constexpr size_t numDecisionVariables = N_ * (nx_ + nu_);
  static constexpr size_t numConstraints = N_ * (nx_ + nc_);

  OcpToKktTest() {
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

TEST_F(OcpToKktTest, sparseConstraintsApproximation) {
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

TEST_F(OcpToKktTest, sparseCostApproximation) {
  Eigen::SparseMatrix<ocs2::scalar_t> H;
  ocs2::vector_t h;

  ocs2::ScalarFunctionQuadraticApproximation costApproximation;
  ocs2::getCostMatrix(ocpSize_, x0, costArray, costApproximation);
  ocs2::getCostMatrixSparse(ocpSize_, x0, costArray, H, h);

  EXPECT_TRUE(costApproximation.dfdxx.isApprox(H.toDense()));
  EXPECT_TRUE(costApproximation.dfdx.isApprox(h));
}
