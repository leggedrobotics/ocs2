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

#include "ocs2_centroidal_model/FactoryFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamics.h"
#include "ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h"
#include "ocs2_centroidal_model/example/anymal/definitions.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

using namespace ocs2;
using namespace centroidal_model;

TEST(AnymalSingleRigidBodyModelTestInit, InitModelFromUrdf) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfPath);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t numJoints = model.nq - 6;
  CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::SingleRigidBodyDynamics,
                                                       anymalInitialState.tail(numJoints), anymal3DofContactNames, anymal6DofContactNames);
  CentroidalModelPinocchioMapping mapping(info);
  mapping.setPinocchioInterface(pinocchioInterface);

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

TEST(AnymalSingleRigidBodyModelTestInit, InitModelFromUrdfAD) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfPath);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t nq = model.nq;
  const size_t numJoints = nq - 6;
  CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::SingleRigidBodyDynamics,
                                                       anymalInitialState.tail(numJoints), anymal3DofContactNames, anymal6DofContactNames);
  CentroidalModelPinocchioMappingCppAd mappingAD(info.toCppAd());
  mappingAD.setPinocchioInterface(pinocchioInterface.toCppAd());
  const auto& centroidalModelInfo = mappingAD.getCentroidalModelInfo();

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(model.nq + 6, anymal::STATE_DIM);
  EXPECT_EQ(model.nq - 6 + 3 * centroidalModelInfo.numThreeDofContacts + 6 * centroidalModelInfo.numSixDofContacts, anymal::INPUT_DIM);
}

class TestAnymalSingleRigidBodyModel : public testing::Test {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  TestAnymalSingleRigidBodyModel() : pinocchioInterface_(createPinocchioInterface(anymalUrdfPath)) {
    size_t nq = pinocchioInterface_.getModel().nq;
    const size_t numJoints = nq - 6;
    CentroidalModelInfo info =
        createCentroidalModelInfo(pinocchioInterface_, CentroidalModelType::SingleRigidBodyDynamics, anymalInitialState.tail(numJoints),
                                  anymal3DofContactNames, anymal6DofContactNames);
    mapping_.reset(new CentroidalModelPinocchioMapping(info));
    mapping_->setPinocchioInterface(pinocchioInterface_);

    anymalKinoCentroidalDynamicsPtr = std::make_shared<PinocchioCentroidalDynamics>(info);
    anymalKinoCentroidalDynamicsPtr->setPinocchioInterface(pinocchioInterface_);

    anymalKinoCentroidalDynamicsAdPtr = std::make_shared<PinocchioCentroidalDynamicsAD>(
        pinocchioInterface_, info, "AnymalSingleRigidBodyTestAD", anymalCppAdModelPath, true, false);
    srand(0);
    time = 0.0;
    state = ocs2::vector_t::Random(anymal::STATE_DIM);
    input = 10000 * ocs2::vector_t::Random(anymal::INPUT_DIM);
  }

  ocs2::scalar_t time;
  ocs2::vector_t state;
  ocs2::vector_t input;

  PinocchioInterface pinocchioInterface_;
  std::unique_ptr<CentroidalModelPinocchioMapping> mapping_;
  std::shared_ptr<PinocchioCentroidalDynamics> anymalKinoCentroidalDynamicsPtr;
  std::shared_ptr<PinocchioCentroidalDynamicsAD> anymalKinoCentroidalDynamicsAdPtr;
};

static void compareApproximation(const ocs2::VectorFunctionLinearApproximation& a, const ocs2::VectorFunctionLinearApproximation& b,
                                 double tol = 1e-6) {
  if (!a.f.isApprox(b.f)) {
    std::cerr << "compare dynamics\n";
    visualMatrixCompare(a.f.transpose(), b.f.transpose(), tol);
  }
  if (!a.dfdx.isApprox(b.dfdx)) {
    std::cerr << "compare dfdx\n";
    visualMatrixCompare(a.dfdx, b.dfdx, tol);
  }
  if (!a.dfdu.isApprox(b.dfdu)) {
    std::cerr << "compare dfdu\n";
    visualMatrixCompare(a.dfdu, b.dfdu, tol);
  }

  EXPECT_TRUE(a.f.isApprox(b.f));
  EXPECT_TRUE(a.dfdx.isApprox(b.dfdx));
  EXPECT_TRUE(a.dfdu.isApprox(b.dfdu));
}

TEST_F(TestAnymalSingleRigidBodyModel, ComputeFlowMap) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const auto dynamics = anymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
}

TEST_F(TestAnymalSingleRigidBodyModel, ComputeLinearApproximation) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const vector_t vPinocchio = mapping_->getPinocchioJointVelocity(state, input);
  updateCentroidalDynamicsDerivatives(pinocchioInterface_, info, qPinocchio, vPinocchio);
  const auto linearApproximation = anymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
}

TEST_F(TestAnymalSingleRigidBodyModel, CompareFlowMaps) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const auto stateDerivative = anymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
  const auto stateDerivativeAd = anymalKinoCentroidalDynamicsAdPtr->getValue(time, state, input);
  EXPECT_TRUE(stateDerivative.isApprox(stateDerivativeAd));
}

TEST_F(TestAnymalSingleRigidBodyModel, CompareFlowMapLinearApproximations) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  updateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const vector_t vPinocchio = mapping_->getPinocchioJointVelocity(state, input);
  updateCentroidalDynamicsDerivatives(pinocchioInterface_, info, qPinocchio, vPinocchio);
  const auto linearApproximation = anymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
  const auto linearApproximationAd = anymalKinoCentroidalDynamicsAdPtr->getLinearApproximation(time, state, input);
  compareApproximation(linearApproximation, linearApproximationAd);
}
