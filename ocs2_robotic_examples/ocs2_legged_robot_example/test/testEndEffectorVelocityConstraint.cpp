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

#include <ocs2_legged_robot_example/LeggedRobotInterface.h>
#include <ocs2_legged_robot_example/constraint/EndEffectorVelocityConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

using namespace ocs2;
using namespace legged_robot;

class testEndEffectorVelocityConstraint : public ::testing::Test {
 public:
  testEndEffectorVelocityConstraint() {
    const std::string urdfPath = ros::package::getPath("anymal_c_simple_description") + "/urdf/anymal.urdf";

    pinocchioInterfacePtr.reset(new PinocchioInterface(LeggedRobotInterface::buildPinocchioInterface(urdfPath)));

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematics(*pinocchioInterfacePtr, *pinocchioMappingPtr, {CONTACT_POINTS_NAMES_[0]}));
    eeKinematicsAdPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr, *pinocchioMappingAdPtr,
                                                                    {CONTACT_POINTS_NAMES_[0]}, STATE_DIM_, INPUT_DIM_, "EEVel"));

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

    u = vector_t::Random(INPUT_DIM_);

    // Zero velocity constraint (without position error gain in z-direction)
    settings.A.resize(3, 6);
    settings.A << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero();
    settings.b = Eigen::Vector3d::Zero();

    const vector_t& qNominal = x.tail(GENERALIZED_VEL_NUM_);
    CentroidalModelInfoTpl<scalar_t> info(*pinocchioInterfacePtr, CentroidalModelType::FullCentroidalDynamics, qNominal,
                                          LEGGED_ROBOT_3_DOF_CONTACT_NAMES_, LEGGED_ROBOT_6_DOF_CONTACT_NAMES_);
    pinocchioMappingPtr.reset(new CentroidalModelPinocchioMapping<scalar_t>(info));

    CentroidalModelInfoTpl<ad_scalar_t> infoAD(*pinocchioInterfacePtr, CentroidalModelType::FullCentroidalDynamics, qNominal,
                                               LEGGED_ROBOT_3_DOF_CONTACT_NAMES_, LEGGED_ROBOT_6_DOF_CONTACT_NAMES_);
    pinocchioMappingAdPtr.reset(new CentroidalModelPinocchioMapping<ad_scalar_t>(infoAD));
  }

  vector_t x{STATE_DIM_};
  vector_t u{INPUT_DIM_};
  EndEffectorVelocityConstraintSettings settings;
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> eeKinematicsAdPtr;
  std::unique_ptr<CentroidalModelPinocchioMapping<scalar_t>> pinocchioMappingPtr;
  std::unique_ptr<CentroidalModelPinocchioMapping<ad_scalar_t>> pinocchioMappingAdPtr;
};

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testAnalytical) {
  using vector3_t = EndEffectorVelocityConstraint::vector3_t;

  auto eeVelConstraintPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsPtr, 3);
  eeVelConstraintPtr->configure(settings);

  dynamic_cast<PinocchioEndEffectorKinematics&>(eeVelConstraintPtr->getEndEffectorKinematics())
      .setPinocchioInterface(*pinocchioInterfacePtr);

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();

  const auto q = pinocchioMappingPtr->getPinocchioJointPosition(x);
  pinocchio::computeCentroidalMap(model, data, q);
  const auto v = pinocchioMappingPtr->getPinocchioJointVelocity(x, vector_t::Zero(INPUT_DIM_));
  const auto a = vector_t::Zero(q.size());

  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  std::cerr << "constraint:\n" << eeVelConstraintPtr->getValue(0.0, x, u) << '\n';
  std::cerr << "approximation:\n" << eeVelConstraintPtr->getLinearApproximation(0.0, x, vector_t::Zero(INPUT_DIM_));
}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testCppAd) {
  auto eeVelConstraintAdPtr = std::make_shared<EndEffectorVelocityConstraint>(*eeKinematicsAdPtr, 3);
  eeVelConstraintAdPtr->configure(settings);

  std::cerr << "constraint:\n" << eeVelConstraintAdPtr->getValue(0.0, x, u) << '\n';
  std::cerr << "approximation:\n" << eeVelConstraintAdPtr->getLinearApproximation(0.0, x, u);
}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testValue) {}

TEST_F(testEndEffectorVelocityConstraint, DISABLED_testLinearApproximation) {}
