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

#include <gtest/gtest.h>
#include <ros/package.h>

#include <ocs2_core/soft_constraint/penalties/QuadraticPenaltyFunction.h>
#include <ocs2_core/soft_constraint/penalties/SmoothAbsolutePenaltyFunction.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorPinocchioMapping.h>
#include <ocs2_mobile_manipulator_example/cost/EndEffectorCost.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

using namespace mobile_manipulator;

class TestEndEffectorKinematics : public ::testing::Test {
 public:
  TestEndEffectorKinematics() {
    const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";

    pinocchioInterfacePtr.reset(new ocs2::PinocchioInterface(MobileManipulatorInterface::buildPinocchioInterface(urdfPath)));

    eeKinematicsPtr.reset(new ocs2::PinocchioEndEffectorKinematics(*pinocchioInterfacePtr, pinocchioMapping, {"WRIST_2"}));

    x << 1.0, 1.0, 0.5, 2.5, -1.0, 1.5, 0.0, 1.0, 0.0;
    vector_t u = vector_t::Zero(INPUT_DIM);
  }

  vector_t x{STATE_DIM};
  vector_t u{INPUT_DIM};
  std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<ocs2::PinocchioEndEffectorKinematics> eeKinematicsPtr;
  MobileManipulatorPinocchioMapping<scalar_t> pinocchioMapping;
};

TEST_F(TestEndEffectorKinematics, testKinematics) {
  const auto q = pinocchioMapping.getPinocchioJointPosition(x);
  pinocchioInterfacePtr->forwardKinematics(q);
  pinocchioInterfacePtr->updateFramePlacements();
  pinocchioInterfacePtr->computeJointJacobians(q);

  const auto id = pinocchioInterfacePtr->getBodyId("WRIST_2");
  vector_t pos = pinocchioInterfacePtr->getBodyPosition(id);

  const auto eePos = eeKinematicsPtr->getPositions(x)[0];
  const auto eePosLin = eeKinematicsPtr->getPositionsLinearApproximation(x)[0];

  EXPECT_TRUE(pos.isApprox(eePos));
  EXPECT_TRUE(pos.isApprox(eePosLin.f));

  std::cerr << "position:\n" << eePos.transpose() << '\n';
  std::cerr << "linear approximation:\n" << eePosLin;
}

TEST_F(TestEndEffectorKinematics, testEndEffectorCost) {
  // ocs2::SmoothAbsolutePenaltyFunction penalty(ocs2::SmoothAbsolutePenaltyFunction::Config(1.0, 1e-2));
  ocs2::QuadraticPenaltyFunction penalty(1.0);

  auto eeCostPtr = std::make_shared<EndEffectorCost>(*eeKinematicsPtr, penalty);
  ocs2::CostDesiredTrajectories initCostDesiredTrajectory({0.0}, {vector_t::Zero(7)}, {vector_t::Zero(INPUT_DIM)});
  eeCostPtr->setCostDesiredTrajectoriesPtr(&initCostDesiredTrajectory);

  std::cerr << "cost:\n" << eeCostPtr->cost(0.0, x, u) << '\n';
  std::cerr << "quadratic approximation:\n" << eeCostPtr->costQuadraticApproximation(0.0, x, u);
}
