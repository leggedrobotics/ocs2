/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <gtest/gtest.h>
#include <ros/package.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <ocs2_legged_robot_example/LeggedRobotInterface.h>
#include <ocs2_legged_robot_example/constraint/EndEffectorVelocityConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

using namespace ocs2;
using namespace legged_robot;

class testEndEffectorVelocityConstraint : public ::testing::Test {
 public:
  testEndEffectorVelocityConstraint() {
    pinocchioInterfacePtr.reset(new PinocchioInterface(LeggedRobotInterface::buildPinocchioInterface(ROBOT_URDF_PATH_)));

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematics(*pinocchioInterfacePtr, *pinocchioMappingPtr, {CONTACT_NAMES_3_DOF_[0]}));
    eeKinematicsAdPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr, *pinocchioMappingAdPtr,
                                                                    {CONTACT_NAMES_3_DOF_[0]}, centroidalModelInfo.stateDim,
                                                                    centroidalModelInfo.inputDim, "EEVel"));

    x(0) = 0.0;  // vcom_x
    x(1) = 0.0;  // vcom_y
    x(2) = 0.0;  // vcom_z
    x(3) = 0.0;  // L_x / robotMass
    x(4) = 0.0;  // L_y / robotMass
    x(5) = 0.0;  // L_z / robotMass

    x(6) = 0.0;   // p_base_x
    x(7) = 0.0;   // p_base_y
    x(8) = 0.57;  // p_base_z
    x(9) = 0.0;   // theta_base_z
    x(10) = 0.0;  // theta_base_y
    x(11) = 0.0;  // theta_base_x

    x(12) = -0.25;  // LF_HAA
    x(13) = 0.6;    // LF_HFE
    x(14) = -0.85;  // LF_KFE
    x(15) = -0.25;  // LH_HAA
    x(16) = -0.6;   // LH_HFE
    x(17) = 0.85;   // LH_KFE
    x(18) = 0.25;   // RF_HAA
    x(19) = 0.6;    // RF_HFE
    x(20) = -0.85;  // RF_KFE
    x(21) = 0.25;   // RH_HAA
    x(22) = -0.6;   // RH_HFE
    x(23) = 0.85;   // RH_KFE

    u = vector_t::Random(centroidalModelInfo.inputDim);

    // Zero velocity constraint (without position error gain in z-direction)
    settings.A.resize(3, 6);
    settings.A << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
    settings.b = Eigen::Vector3d::Zero();

    pinocchioMappingPtr.reset(new CentroidalModelPinocchioMapping<scalar_t>(centroidalModelInfo));
    pinocchioMappingAdPtr.reset(new CentroidalModelPinocchioMapping<ad_scalar_t>(centroidalModelInfoAd));
  }

  vector_t x{centroidalModelInfo.stateDim};
  vector_t u{centroidalModelInfo.inputDim};
  EndEffectorVelocityConstraintSettings settings;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsAdPtr;
  std::unique_ptr<CentroidalModelPinocchioMapping<scalar_t>> pinocchioMappingPtr;
  std::unique_ptr<CentroidalModelPinocchioMapping<ad_scalar_t>> pinocchioMappingAdPtr;
};

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testAnalytical) {
  auto eeVelConstraintPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsPtr, 3);
  eeVelConstraintPtr->configure(settings);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  ocs2::updateCentroidalDynamics(*pinocchioInterfacePtr, centroidalModelInfo, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, vector_t::Zero(centroidalModelInfo.inputDim));
  const auto a = vector_t::Zero(q.size());

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  ocs2::updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, centroidalModelInfo, q, v);

  std::cerr << "constraint:\n" << eeVelConstraintPtr->getValue(0.0, x, u) << '\n';
  std::cerr << "approximation:\n" << eeVelConstraintPtr->getLinearApproximation(0.0, x, vector_t::Zero(centroidalModelInfo.inputDim));
}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testCppAd) {
  auto eeVelConstraintAdPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsAdPtr, 3);
  eeVelConstraintAdPtr->configure(settings);

  std::cerr << "constraint:\n" << eeVelConstraintAdPtr->getValue(0.0, x, u) << '\n';
  std::cerr << "approximation:\n" << eeVelConstraintAdPtr->getLinearApproximation(0.0, x, u);
}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testValue) {
  auto eeVelConstraintPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsPtr, 3);
  eeVelConstraintPtr->configure(settings);
  auto eeVelConstraintAdPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsAdPtr, 3);
  eeVelConstraintAdPtr->configure(settings);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  ocs2::updateCentroidalDynamics(*pinocchioInterfacePtr, centroidalModelInfo, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, vector_t::Zero(centroidalModelInfo.inputDim));
  const auto a = vector_t::Zero(q.size());

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  ocs2::updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, centroidalModelInfo, q, v);

  const auto value = eeVelConstraintPtr->getValue(0.0, x, u);
  const auto valueAd = eeVelConstraintAdPtr->getValue(0.0, x, u);
  EXPECT_TRUE(value.isApprox(valueAd));
}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testLinearApproximation) {
  auto eeVelConstraintPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsPtr, 3);
  eeVelConstraintPtr->configure(settings);
  auto eeVelConstraintAdPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsAdPtr, 3);
  eeVelConstraintAdPtr->configure(settings);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  ocs2::updateCentroidalDynamics(*pinocchioInterfacePtr, centroidalModelInfo, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, vector_t::Zero(centroidalModelInfo.inputDim));
  const auto a = vector_t::Zero(q.size());

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);
  ocs2::updateCentroidalDynamicsDerivatives(*pinocchioInterfacePtr, centroidalModelInfo, q, v);

  const auto linApprox = eeVelConstraintPtr->getLinearApproximation(0.0, x, u);
  const auto linApproxAd = eeVelConstraintAdPtr->getLinearApproximation(0.0, x, u);
  EXPECT_TRUE(linApprox.f.isApprox(linApproxAd.f));
  EXPECT_TRUE(linApprox.dfdx.isApprox(linApproxAd.dfdx));
  EXPECT_TRUE(linApprox.dfdu.isApprox(linApproxAd.dfdu));
}
