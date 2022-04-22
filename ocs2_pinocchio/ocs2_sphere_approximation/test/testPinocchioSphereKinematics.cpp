/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_pinocchio_interface/urdf.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematicsCppAd.h>

#include <ocs2_robotic_assets/package_path.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ocs2_pinocchio_interface/urdf.h>

#include <gtest/gtest.h>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;

template <typename SCALAR>
class DummyMapping final : public ocs2::PinocchioStateInputMapping<SCALAR> {
 public:
  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  DummyMapping() = default;
  ~DummyMapping() override = default;
  DummyMapping<SCALAR>* clone() const override { return new DummyMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override { return input; }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    return {Jq, Jv};
  }
};

class TestSphereKinematics : public ::testing::Test {
 public:
  using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

  TestSphereKinematics() {
    const std::string urdfFile = ocs2::robotic_assets::getPath() + "/resources/mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf";
    pinocchioInterfacePtr.reset(new ocs2::PinocchioInterface(ocs2::getPinocchioInterfaceFromUrdfFile(urdfFile)));
    pinocchioSphereInterfacePtr.reset(new ocs2::PinocchioSphereInterface(*pinocchioInterfacePtr, {"ARM", "SHOULDER", "FOREARM", "WRIST_1"},
                                                                         {0.20, 0.10, 0.05, 0.05}, 0.7));
    sphereKinematicsPtr.reset(new ocs2::PinocchioSphereKinematics(*pinocchioSphereInterfacePtr, pinocchioMapping));
    sphereKinematicsCppAdPtr.reset(new ocs2::PinocchioSphereKinematicsCppAd(
        *pinocchioInterfacePtr, *pinocchioSphereInterfacePtr, pinocchioMappingCppAd, pinocchioInterfacePtr->getModel().njoints, 0,
        "pinocchio_sphere_kinematics", "/tmp/ocs2", true, true));

    x.resize(pinocchioInterfacePtr->getModel().njoints);
    // taken form config/mpc/task.info
    x(0) = 2.5;   // SH_ROT
    x(1) = -1.0;  // SH_FLE
    x(2) = 1.5;   // EL_FLE
    x(3) = 0.0;   // EL_ROT
    x(4) = 1.0;   // WR_FLE
    x(5) = 0.0;   // WR_ROT

    q = pinocchioMapping.getPinocchioJointPosition(x);
  }

  void compareApproximation(const ocs2::VectorFunctionLinearApproximation& f1, const ocs2::VectorFunctionLinearApproximation& f2,
                            bool functionOfInput = false) {
    if (!f1.f.isApprox(f2.f)) {
      std::cerr << "f1.f  " << f1.f.transpose() << '\n';
      std::cerr << "f2.f  " << f2.f.transpose() << '\n';
    }

    if (!f1.dfdx.isApprox(f2.dfdx)) {
      std::cerr << "f1.dfdx\n" << f1.dfdx << '\n';
      std::cerr << "f2.dfdx\n" << f2.dfdx << '\n';
    }

    if (functionOfInput && !f1.dfdu.isApprox(f2.dfdu)) {
      std::cerr << "f1.dfdu\n" << f1.dfdu << '\n';
      std::cerr << "f2.dfdu\n" << f2.dfdu << '\n';
    }

    EXPECT_TRUE(f1.f.isApprox(f2.f));
    EXPECT_TRUE(f1.dfdx.isApprox(f2.dfdx));
  }

  ocs2::vector_t x;  // state
  ocs2::vector_t q;  // pinocchio joint positions

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<ocs2::PinocchioSphereInterface> pinocchioSphereInterfacePtr;
  std::unique_ptr<ocs2::PinocchioSphereKinematics> sphereKinematicsPtr;
  std::unique_ptr<ocs2::PinocchioSphereKinematicsCppAd> sphereKinematicsCppAdPtr;
  DummyMapping<ocs2::scalar_t> pinocchioMapping;
  DummyMapping<ocs2::ad_scalar_t> pinocchioMappingCppAd;
};

TEST_F(TestSphereKinematics, testKinematicsPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  const auto& geometryModel = pinocchioSphereInterfacePtr->getGeometryModel();
  const auto geomObjIds = pinocchioSphereInterfacePtr->getGeomObjIds();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, q);

  const auto parentJointId = geometryModel.geometryObjects[geomObjIds[0]].parentJoint;
  const auto& placement = geometryModel.geometryObjects[geomObjIds[0]].placement;
  const vector3_t pos =
      data.oMi[parentJointId].rotation() *
          (placement.rotation() * pinocchioSphereInterfacePtr->getSphereCentersToObjectCenter(0)[0] + placement.translation()) +
      data.oMi[parentJointId].translation();

  const vector3_t sphereOffset = pos - data.oMi[parentJointId].translation();
  ocs2::matrix_t jointJacobian = ocs2::matrix_t::Zero(6, model.nv);
  pinocchio::getJointJacobian(model, data, parentJointId, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jointJacobian);
  const ocs2::matrix_t sphereJq = jointJacobian.topRows<3>() - ocs2::skewSymmetricMatrix(sphereOffset) * jointJacobian.bottomRows<3>();
  ocs2::matrix_t sphereJx;
  std::tie(sphereJx, std::ignore) = pinocchioMapping.getOcs2Jacobian(x, sphereJq.topRows<3>(), ocs2::matrix_t::Zero(0, model.nq));

  sphereKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);
  const vector3_t spherePos = sphereKinematicsPtr->getPosition(x)[0];
  const ocs2::VectorFunctionLinearApproximation spherePosLin = sphereKinematicsPtr->getPositionLinearApproximation(x)[0];

  EXPECT_TRUE(spherePos.isApprox(spherePos));
  EXPECT_TRUE(spherePos.isApprox(spherePosLin.f));
  EXPECT_TRUE(sphereJx.isApprox(spherePosLin.dfdx));
}

TEST_F(TestSphereKinematics, testPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  sphereKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePos = sphereKinematicsPtr->getPosition(x)[0];
  const auto spherePosAd = sphereKinematicsCppAdPtr->getPosition(x)[0];
  EXPECT_TRUE(spherePos.isApprox(spherePosAd));
}

TEST_F(TestSphereKinematics, testPositionApproximation) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  sphereKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePosLin = sphereKinematicsPtr->getPositionLinearApproximation(x)[0];
  const auto spherePosLinAd = sphereKinematicsCppAdPtr->getPositionLinearApproximation(x)[0];
  compareApproximation(spherePosLin, spherePosLinAd);
}

TEST_F(TestSphereKinematics, testClone) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  auto clonePtr = std::unique_ptr<ocs2::PinocchioSphereKinematics>(sphereKinematicsPtr->clone());
  auto cloneCppAdPtr = std::unique_ptr<ocs2::PinocchioSphereKinematicsCppAd>(sphereKinematicsCppAdPtr->clone());

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  clonePtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto spherePos = clonePtr->getPosition(x)[0];
  const auto spherePosAd = cloneCppAdPtr->getPosition(x)[0];
  EXPECT_TRUE(spherePos.isApprox(spherePosAd));
}