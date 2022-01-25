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

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_centroidal_model/CentroidalModelRbdConversions.h"
#include "ocs2_centroidal_model/FactoryFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamics.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h"

#include "ocs2_centroidal_model/test/definitions.h"

using namespace ocs2;
using namespace centroidal_model;

class TestAnymalCentroidalModel : public ::testing::TestWithParam<CentroidalModelType> {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  TestAnymalCentroidalModel() {
    srand(0);
    pinocchioInterfacePtr.reset(new PinocchioInterface(createPinocchioInterface(anymalUrdfFile)));
  }

  CentroidalModelInfo createInfo(CentroidalModelType type) const {
    const size_t nq = pinocchioInterfacePtr->getModel().nq;
    const size_t numJoints = nq - 6;
    return createCentroidalModelInfo(*pinocchioInterfacePtr, type, getInitialState().tail(numJoints), anymal3DofContactNames,
                                     anymal6DofContactNames);
  }

  std::unique_ptr<CentroidalModelPinocchioMapping> createMapping(CentroidalModelType type) const {
    std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr(new CentroidalModelPinocchioMapping(createInfo(type)));
    mappingPtr->setPinocchioInterface(*pinocchioInterfacePtr);
    return mappingPtr;
  }

  static constexpr scalar_t tol = 1e-9;
  static constexpr size_t numTests = 100;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
};

constexpr scalar_t TestAnymalCentroidalModel::tol;
constexpr size_t TestAnymalCentroidalModel::numTests;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, info) {
  const CentroidalModelType type = GetParam();
  const auto info = createInfo(type);
  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, dynamis_flowMap) {
  const CentroidalModelType type = GetParam();
  auto mappingPtr = createMapping(type);
  const auto& info = mappingPtr->getCentroidalModelInfo();

  // Analytical model
  PinocchioCentroidalDynamics anymalDynamics(createInfo(type));
  anymalDynamics.setPinocchioInterface(*pinocchioInterfacePtr);

  // CppAD model
  const std::string modelName = "TestAnymal" + toString(type) + "Ad";
  PinocchioCentroidalDynamicsAD anymalDynamicsAd(*pinocchioInterfacePtr, createInfo(type), modelName);

  for (size_t i = 0; i < numTests; i++) {
    const scalar_t time = 0.0;
    const vector_t state = 10.0 * vector_t::Random(anymal::STATE_DIM);
    const vector_t input = 10000.0 * vector_t::Random(anymal::INPUT_DIM);

    const vector_t qPinocchio = mappingPtr->getPinocchioJointPosition(state);
    updateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);

    const auto stateDerivative = anymalDynamics.getValue(time, state, input);
    const auto stateDerivativeAd = anymalDynamicsAd.getValue(time, state, input);

    const vector_t vPinocchio = mappingPtr->getPinocchioJointVelocity(state, input);
    updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, info, qPinocchio, vPinocchio);

    const auto linearApproximation = anymalDynamics.getLinearApproximation(time, state, input);
    const auto linearApproximationAd = anymalDynamicsAd.getLinearApproximation(time, state, input);

    EXPECT_TRUE(stateDerivative.isApprox(stateDerivativeAd, tol));
    EXPECT_TRUE(stateDerivative.isApprox(linearApproximation.f, tol));
    EXPECT_TRUE(linearApproximationAd.f.isApprox(linearApproximation.f, tol));
    EXPECT_TRUE(linearApproximationAd.dfdx.isApprox(linearApproximation.dfdx, tol));
    EXPECT_TRUE(linearApproximationAd.dfdu.isApprox(linearApproximation.dfdu, tol));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TEST_P(TestAnymalCentroidalModel, rbd_conversion) {
  const CentroidalModelType type = GetParam();
  const auto info = createInfo(type);

  // Analytical model
  CentroidalModelRbdConversions rbdConversions(*pinocchioInterfacePtr, info);

  // ocs2 --> rbd --> ocs2
  for (size_t i = 0; i < numTests; i++) {
    const vector_t state = 10.0 * vector_t::Random(anymal::STATE_DIM);
    const vector_t input = 10000.0 * vector_t::Random(anymal::INPUT_DIM);

    const vector_t rbdState = rbdConversions.computeRbdStateFromCentroidalModel(state, input);
    const vector_t reconstructedState = rbdConversions.computeCentroidalStateFromRbdModel(rbdState);

    EXPECT_TRUE(state.isApprox(reconstructedState, tol));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
INSTANTIATE_TEST_CASE_P(TestAnymalCentroidalModelWithParam, TestAnymalCentroidalModel,
                        testing::ValuesIn({CentroidalModelType::FullCentroidalDynamics, CentroidalModelType::SingleRigidBodyDynamics}),
                        [](const testing::TestParamInfo<TestAnymalCentroidalModel::ParamType>& info) { return toString(info.param); });
