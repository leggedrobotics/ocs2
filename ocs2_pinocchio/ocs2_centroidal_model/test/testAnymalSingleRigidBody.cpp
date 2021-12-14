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

#include "ocs2_centroidal_model/FactoryFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamics.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h"

#include "ocs2_centroidal_model/test/definitions.h"

using namespace ocs2;
using namespace centroidal_model;

class TestAnymalSingleRigidBodyModel : public testing::Test {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  TestAnymalSingleRigidBodyModel() {
    pinocchioInterfacePtr.reset(new PinocchioInterface(createPinocchioInterface(anymalUrdfFile)));

    size_t nq = pinocchioInterfacePtr->getModel().nq;
    const size_t numJoints = nq - 6;
    CentroidalModelInfo info = createCentroidalModelInfo(*pinocchioInterfacePtr, CentroidalModelType::SingleRigidBodyDynamics,
                                                         getInitialState().tail(numJoints), anymal3DofContactNames, anymal6DofContactNames);
    mappingPtr.reset(new CentroidalModelPinocchioMapping(info));
    mappingPtr->setPinocchioInterface(*pinocchioInterfacePtr);

    anymalKinoCentroidalDynamicsPtr = std::make_shared<PinocchioCentroidalDynamics>(info);
    anymalKinoCentroidalDynamicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

    anymalKinoCentroidalDynamicsAdPtr =
        std::make_shared<PinocchioCentroidalDynamicsAD>(*pinocchioInterfacePtr, info, "AnymalSingleRigidBodyTestAD");
    srand(0);
    time = 0.0;
    state = ocs2::vector_t::Random(anymal::STATE_DIM);
    input = 10000 * ocs2::vector_t::Random(anymal::INPUT_DIM);
  }

  ocs2::scalar_t time;
  ocs2::vector_t state;
  ocs2::vector_t input;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<CentroidalModelPinocchioMapping> mappingPtr;
  std::shared_ptr<PinocchioCentroidalDynamics> anymalKinoCentroidalDynamicsPtr;
  std::shared_ptr<PinocchioCentroidalDynamicsAD> anymalKinoCentroidalDynamicsAdPtr;
};

TEST_F(TestAnymalSingleRigidBodyModel, ComputeFlowMap) {
  EXPECT_NO_THROW({
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();
    const auto& info = mappingPtr->getCentroidalModelInfo();
    const vector_t qPinocchio = mappingPtr->getPinocchioJointPosition(state);
    updateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);
    const auto dynamics = anymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
  });
}

TEST_F(TestAnymalSingleRigidBodyModel, ComputeLinearApproximation) {
  EXPECT_NO_THROW({
    const auto& model = pinocchioInterfacePtr->getModel();
    auto& data = pinocchioInterfacePtr->getData();
    const auto& info = mappingPtr->getCentroidalModelInfo();
    const vector_t qPinocchio = mappingPtr->getPinocchioJointPosition(state);
    updateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);
    const vector_t vPinocchio = mappingPtr->getPinocchioJointVelocity(state, input);
    updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, info, qPinocchio, vPinocchio);
    const auto linearApproximation = anymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
  });
}

TEST_F(TestAnymalSingleRigidBodyModel, CompareFlowMaps) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  const auto& info = mappingPtr->getCentroidalModelInfo();
  const vector_t qPinocchio = mappingPtr->getPinocchioJointPosition(state);
  updateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);
  const auto stateDerivative = anymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
  const auto stateDerivativeAd = anymalKinoCentroidalDynamicsAdPtr->getValue(time, state, input);
  EXPECT_TRUE(stateDerivative.isApprox(stateDerivativeAd));
}

TEST_F(TestAnymalSingleRigidBodyModel, CompareFlowMapLinearApproximations) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  const auto& info = mappingPtr->getCentroidalModelInfo();
  const vector_t qPinocchio = mappingPtr->getPinocchioJointPosition(state);
  updateCentroidalDynamics(*pinocchioInterfacePtr, info, qPinocchio);
  const vector_t vPinocchio = mappingPtr->getPinocchioJointVelocity(state, input);
  updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, info, qPinocchio, vPinocchio);
  const auto linearApproximation = anymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
  const auto linearApproximationAd = anymalKinoCentroidalDynamicsAdPtr->getLinearApproximation(time, state, input);
  compareApproximation(linearApproximation, linearApproximationAd);
}
