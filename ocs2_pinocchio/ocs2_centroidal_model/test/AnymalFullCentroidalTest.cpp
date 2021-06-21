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

#include <ocs2_centroidal_model/example/anymal/AnymalKinoCentroidalDynamics.h>
#include <ocs2_centroidal_model/example/anymal/AnymalKinoCentroidalDynamicsAD.h>
#include <ocs2_centroidal_model/example/anymal/definitions.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/centroidal-derivatives.hpp>

using namespace ocs2;

static PinocchioInterface getAnymalPinocchioInterface() {
  // build a joint model having just 2 joints: one translational joint and one spherical joint
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  PinocchioInterface pinocchioInterface(getPinocchioInterfaceFromUrdfFile(anymalUrdfPath, jointComposite));
  return pinocchioInterface;
}

TEST(AnymalFullCentroidalModelTestInit, InitModelFromUrdf) {
  PinocchioInterface pinocchioInterface = getAnymalPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t nq = model.nq;
  CentroidalModelInfoTpl<scalar_t> info(pinocchioInterface,
                                        CentroidalModelType::FullCentroidalDynamics,
                                        vector_t::Zero(nq), anymal3DofContactNames,
                                        anymal6DofContactNames);
  CentroidalModelPinocchioMapping<scalar_t> mapping(info);
  mapping.setPinocchioInterface(pinocchioInterface);

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

TEST(AnymalFullCentroidalModelTestInit, InitModelFromUrdfAD) {
  PinocchioInterface pinocchioInterface = getAnymalPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t nq = model.nq;
  CentroidalModelInfoTpl<ad_scalar_t> infoAD(pinocchioInterface,
                                             CentroidalModelType::FullCentroidalDynamics,
                                             vector_t::Zero(nq), anymal3DofContactNames,
                                             anymal6DofContactNames);
  CentroidalModelPinocchioMapping<ad_scalar_t> mappingAD(infoAD);
  mappingAD.setPinocchioInterface(pinocchioInterface.toCppAd());

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(infoAD.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(infoAD.inputDim, anymal::INPUT_DIM);
}

class AnymalFullCentroidalModelTest : public testing::Test {
 public:
  using Matrix6x = Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>;
  AnymalFullCentroidalModelTest() : pinocchioInterface_(getAnymalPinocchioInterface()) {
    size_t nq = pinocchioInterface_.getModel().nq;
    CentroidalModelInfoTpl<scalar_t> info(pinocchioInterface_,
                                          CentroidalModelType::FullCentroidalDynamics,
                                          vector_t::Zero(nq), anymal3DofContactNames,
                                          anymal6DofContactNames);
    mapping_.reset(new CentroidalModelPinocchioMapping<scalar_t>(info));
    AnymalKinoCentroidalDynamicsPtr = std::make_shared<AnymalKinoCentroidalDynamics>(pinocchioInterface_, *mapping_);

    CentroidalModelInfoTpl<ad_scalar_t> infoAD(pinocchioInterface_,
                                          CentroidalModelType::FullCentroidalDynamics,
                                          vector_t::Zero(nq), anymal3DofContactNames,
                                          anymal6DofContactNames);
    mappingAD_.reset(new CentroidalModelPinocchioMapping<ad_scalar_t>(infoAD));
    AnymalKinoCentroidalDynamicsAdPtr = std::make_shared<AnymalKinoCentroidalDynamicsAD>(pinocchioInterface_, *mappingAD_);

    srand(0);
    time = 0.0;
    state = ocs2::vector_t::Random(anymal::STATE_DIM);
    input = ocs2::vector_t::Random(anymal::INPUT_DIM);
  }

  ocs2::scalar_t time;
  ocs2::vector_t state;
  ocs2::vector_t input;

  PinocchioInterface pinocchioInterface_;
  std::unique_ptr<CentroidalModelPinocchioMapping<scalar_t>> mapping_;
  std::unique_ptr<CentroidalModelPinocchioMapping<ad_scalar_t>> mappingAD_;
  std::shared_ptr<AnymalKinoCentroidalDynamics> AnymalKinoCentroidalDynamicsPtr;
  std::shared_ptr<AnymalKinoCentroidalDynamicsAD> AnymalKinoCentroidalDynamicsAdPtr;

  // centroidal momentum derivatives
  Matrix6x dh_dq_;
  Matrix6x dhdot_dq_;
  Matrix6x dhdot_dv_;
  Matrix6x dhdot_da_;
};

static void visualMatrixCompare(const ocs2::matrix_t& A, const ocs2::matrix_t& B, double tol = 1e-6) {
  if (A.rows() != B.rows() || A.cols() != B.cols()) {
    std::cerr << "Matrices are not of same size\n";
  }
  for (int row = 0; row < A.rows(); row++) {
    for (int col = 0; col < A.cols(); col++) {
      const double error = std::abs(A(row, col) - B(row, col));
      if (error < tol) {
        std::cerr << " ";
      } else {
        std::cerr << " (" << row << ", " << col << "): " << error;
      }
    }
    std::cerr << '\n';
  }
}

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
  // EXPECT_TRUE(a.dfdx.isApprox(b.dfdx));
  // The gradients of the base pose time derivatives are not correct due to a wrong dh_dq_ in pinocchio
  EXPECT_TRUE(a.dfdx.topRows<6>().isApprox(b.dfdx.topRows<6>()));
  EXPECT_TRUE(a.dfdx.bottomRows<12>().isApprox(b.dfdx.bottomRows<12>()));
  EXPECT_TRUE(a.dfdx.leftCols<9>().isApprox(b.dfdx.leftCols<9>()));
  EXPECT_TRUE(a.dfdu.isApprox(b.dfdu));
}

TEST_F(AnymalFullCentroidalModelTest, ComputeFlowMap) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  pinocchio::computeCentroidalMap(model, data, qPinocchio);
  pinocchio::updateFramePlacements(model, data);
  const auto dynamics = AnymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
}

TEST_F(AnymalFullCentroidalModelTest, ComputeLinearApproximation) {
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  pinocchio::computeCentroidalMap(model, data, qPinocchio);
  const vector_t vPinocchio = mapping_->getPinocchioJointVelocity(state, input);
  dh_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dv_.resize(6, info.generalizedCoordinatesNum);
  dhdot_da_.resize(6, info.generalizedCoordinatesNum);
  pinocchio::computeCentroidalDynamicsDerivatives(model, data,
                                                  qPinocchio, vPinocchio,
                                                  vector_t::Zero(info.generalizedCoordinatesNum),
                                                  dh_dq_, dhdot_dq_, dhdot_dv_, dhdot_da_);
  pinocchio::updateFramePlacements(model, data);
  const auto linearApproximation = AnymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
}

TEST_F(AnymalFullCentroidalModelTest, CompareFlowMaps) {
  const auto stateDerivativeAd = AnymalKinoCentroidalDynamicsAdPtr->getValue(time, state, input);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  pinocchio::computeCentroidalMap(model, data, qPinocchio);
  pinocchio::updateFramePlacements(model, data);
  const auto stateDerivative = AnymalKinoCentroidalDynamicsPtr->getValue(time, state, input);
  EXPECT_TRUE(stateDerivative.isApprox(stateDerivativeAd));
}

TEST_F(AnymalFullCentroidalModelTest, CompareFlowMapLinearApproximations) {
  const auto linearApproximationAd = AnymalKinoCentroidalDynamicsAdPtr->getLinearApproximation(time, state, input);
  const auto& model = pinocchioInterface_.getModel();
  auto& data = pinocchioInterface_.getData();
  const auto& info = mapping_->getCentroidalModelInfo();
  const vector_t qPinocchio = mapping_->getPinocchioJointPosition(state);
  pinocchio::computeCentroidalMap(model, data, qPinocchio);
  const vector_t vPinocchio = mapping_->getPinocchioJointVelocity(state, input);
  dh_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dq_.resize(6, info.generalizedCoordinatesNum);
  dhdot_dv_.resize(6, info.generalizedCoordinatesNum);
  dhdot_da_.resize(6, info.generalizedCoordinatesNum);
  pinocchio::computeCentroidalDynamicsDerivatives(model, data,
                                                  qPinocchio, vPinocchio,
                                                  vector_t::Zero(info.generalizedCoordinatesNum),
                                                  dh_dq_, dhdot_dq_, dhdot_dv_, dhdot_da_);
  pinocchio::updateFramePlacements(model, data);
  const auto linearApproximation = AnymalKinoCentroidalDynamicsPtr->getLinearApproximation(time, state, input);
  compareApproximation(linearApproximation, linearApproximationAd);
}
