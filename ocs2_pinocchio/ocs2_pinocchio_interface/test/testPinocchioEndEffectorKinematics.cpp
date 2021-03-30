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

#include <pinocchio/algorithm/frames-derivatives.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/urdf.h>

#include <ocs2_core/automatic_differentiation/FiniteDifferenceMethods.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <gtest/gtest.h>

#include "ManipulatorArmUrdf.h"

template <typename SCALAR>
class ManipulatorMapping final : public ocs2::PinocchioStateInputMapping<SCALAR> {
 public:
  using scalar_t = SCALAR;
  using vector_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;
  using matrix_t = Eigen::Matrix<SCALAR, Eigen::Dynamic, Eigen::Dynamic>;

  ManipulatorMapping() = default;
  ~ManipulatorMapping() override = default;
  ManipulatorMapping<SCALAR>* clone() const override { return new ManipulatorMapping<SCALAR>(*this); }

  vector_t getPinocchioJointPosition(const vector_t& state) const override { return state; }

  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override { return input; }

  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    return {Jq, Jv};
  }
};

class TestEndEffectorKinematics : public ::testing::Test {
 public:
  using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

  TestEndEffectorKinematics() {
    auto pinocchioInterface = ocs2::getPinocchioInterfaceFromUrdfString(manipulatorArmUrdf);

    pinocchioInterfacePtr.reset(new ocs2::PinocchioInterface(pinocchioInterface));
    eeKinematicsPtr.reset(new ocs2::PinocchioEndEffectorKinematics(pinocchioInterface, pinocchioMapping, {"WRIST_2"}));
    eeKinematicsCppAdPtr.reset(new ocs2::PinocchioEndEffectorKinematicsCppAd(
        pinocchioInterface, pinocchioMappingCppAd, {"WRIST_2"}, 6, 6, "pinocchio_end_effector_kinematics", "/tmp/ocs2", true, false));

    x.resize(6);
    x << 2.5, -1.0, 1.5, 0.0, 1.0, 0.0;
    u.setOnes(6);

    q = pinocchioMapping.getPinocchioJointPosition(x);
    v = pinocchioMapping.getPinocchioJointVelocity(x, u);
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
    if (functionOfInput) {
      EXPECT_TRUE(f1.dfdu.isApprox(f2.dfdu));
    }
  }

  ocs2::vector_t x;  // state
  ocs2::vector_t u;  // input
  ocs2::vector_t q;  // pinocchio joint positions
  ocs2::vector_t v;  // pinocchio joint velocities

  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> eeKinematicsCppAdPtr;
  ManipulatorMapping<ocs2::scalar_t> pinocchioMapping;
  ManipulatorMapping<ocs2::ad_scalar_t> pinocchioMappingCppAd;
};

TEST_F(TestEndEffectorKinematics, testKinematicsPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  const auto id = model.getBodyId("WRIST_2");
  const ocs2::vector_t pos = data.oMf[id].translation();
  ocs2::matrix_t J = ocs2::matrix_t::Zero(6, model.nq);
  pinocchio::getFrameJacobian(model, data, id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);
  const auto eePos = eeKinematicsPtr->getPosition(x)[0];
  const auto eePosLin = eeKinematicsPtr->getPositionLinearApproximation(x)[0];

  EXPECT_TRUE(pos.isApprox(eePos));
  EXPECT_TRUE(pos.isApprox(eePosLin.f));
  EXPECT_TRUE(J.topRows<3>().isApprox(eePosLin.dfdx));
}

TEST_F(TestEndEffectorKinematics, testPosition) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePos = eeKinematicsPtr->getPosition(x)[0];
  const auto eePosAd = eeKinematicsCppAdPtr->getPosition(x)[0];
  EXPECT_TRUE(eePos.isApprox(eePosAd));
}

TEST_F(TestEndEffectorKinematics, testPositionApproximation) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePosLin = eeKinematicsPtr->getPositionLinearApproximation(x)[0];
  const auto eePosLinAd = eeKinematicsCppAdPtr->getPositionLinearApproximation(x)[0];
  compareApproximation(eePosLin, eePosLinAd);
}

TEST_F(TestEndEffectorKinematics, testVelocity) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto a = ocs2::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeVel = eeKinematicsPtr->getVelocity(x, u)[0];
  const auto eeVelAd = eeKinematicsCppAdPtr->getVelocity(x, u)[0];
  EXPECT_TRUE(eeVel.isApprox(eeVelAd));
}

TEST_F(TestEndEffectorKinematics, testVelocityApproximation) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto a = ocs2::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeVelLin = eeKinematicsPtr->getVelocityLinearApproximation(x, u)[0];
  const auto eeVelLinAd = eeKinematicsCppAdPtr->getVelocityLinearApproximation(x, u)[0];
  compareApproximation(eeVelLin, eeVelLinAd, /* functionOfInput = */ true);
}

TEST_F(TestEndEffectorKinematics, testOrientationError) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const quaternion_t qRef(1, 0, 0, 0);
  const auto eeOrientationError = eeKinematicsPtr->getOrientationError(x, {qRef})[0];
  const auto eeOrientationErrorAd = eeKinematicsCppAdPtr->getOrientationError(x, {qRef})[0];
  EXPECT_TRUE(eeOrientationError.isApprox(eeOrientationErrorAd));
}

TEST_F(TestEndEffectorKinematics, testOrientationErrorApproximation) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const quaternion_t qRef(1, 0, 0, 0);
  const auto eeOrientationErrorLin = eeKinematicsPtr->getOrientationErrorLinearApproximation(x, {qRef})[0];
  const auto eeOrientationErrorLinAd = eeKinematicsCppAdPtr->getOrientationErrorLinearApproximation(x, {qRef})[0];
  compareApproximation(eeOrientationErrorLin, eeOrientationErrorLinAd);
}

TEST_F(TestEndEffectorKinematics, testOrientationError_zeroError) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const auto frameId = model.getBodyId("WRIST_2");
  const quaternion_t qRef = ocs2::matrixToQuaternion(data.oMf[frameId].rotation());
  eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eeOrientationError = eeKinematicsPtr->getOrientationError(x, {qRef})[0];
  const auto eeOrientationErrorAd = eeKinematicsCppAdPtr->getOrientationError(x, {qRef})[0];

  EXPECT_TRUE(eeOrientationError.norm() < 1e-9);
  EXPECT_TRUE(eeOrientationErrorAd.norm() < 1e-9);
}

TEST_F(TestEndEffectorKinematics, testClone) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  auto clonePtr = std::unique_ptr<ocs2::PinocchioEndEffectorKinematics>(eeKinematicsPtr->clone());
  auto cloneCppAdPtr = std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd>(eeKinematicsCppAdPtr->clone());

  const auto a = ocs2::vector_t::Zero(q.rows());
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  pinocchio::updateFramePlacements(model, data);

  clonePtr->setPinocchioInterface(*pinocchioInterfacePtr);

  const auto eePos = clonePtr->getPosition(x)[0];
  const auto eePosAd = cloneCppAdPtr->getPosition(x)[0];
  EXPECT_TRUE(eePos.isApprox(eePosAd));

  const auto eeVel = clonePtr->getVelocity(x, u)[0];
  const auto eeVelAd = cloneCppAdPtr->getVelocity(x, u)[0];
  EXPECT_TRUE(eeVel.isApprox(eeVelAd));
}

/* Test to understand the frame jacobian */
TEST_F(TestEndEffectorKinematics, testPinocchioOrientationErrorJacoiban) {
  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  const auto v = ocs2::vector_t::Zero(model.nv);
  const auto frameId = model.getBodyId("WRIST_2");
  const quaternion_t qRef(1, 0, 0, 0);
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const ocs2::scalar_t eps = 1e-9;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  // Check geometric jacobian
  ocs2::matrix_t J = ocs2::matrix_t::Zero(6, model.nq);
  pinocchio::getFrameJacobian(model, data, frameId, rf, J);

  auto func1 = [&](const ocs2::vector_t& v) -> ocs2::vector_t {
    pinocchio::forwardKinematics(model, data, x, v);
    pinocchio::updateFramePlacements(model, data);
    return getFrameVelocity(model, data, frameId, rf).angular();
  };
  const ocs2::matrix_t Jgeom_fd = ocs2::finiteDifferenceDerivative(func1, v, eps);
  EXPECT_TRUE(J.bottomRows<3>().isApprox(Jgeom_fd, sqrt(eps)));

  // Check analytic jacobian
  quaternion_t q = ocs2::matrixToQuaternion(data.oMf[frameId].rotation());
  ocs2::matrix_t Janalytic =
      (ocs2::quaternionDistanceJacobian(q, qRef) * ocs2::angularVelocityToQuaternionTimeDerivative(q)) * J.bottomRows<3>();

  auto func2 = [&](const ocs2::vector_t& x) -> ocs2::vector_t {
    const auto q = pinocchioMapping.getPinocchioJointPosition(x);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    eeKinematicsPtr->setPinocchioInterface(*pinocchioInterfacePtr);
    return eeKinematicsPtr->getOrientationError(x, {qRef})[0];
  };
  const ocs2::matrix_t Janalytic_fd = ocs2::finiteDifferenceDerivative(func2, x, eps);
  EXPECT_TRUE(Janalytic.isApprox(Janalytic_fd, sqrt(eps)));
}
