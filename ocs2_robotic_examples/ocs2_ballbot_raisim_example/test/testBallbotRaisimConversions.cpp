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
#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>

#include <ocs2_ballbot_example/definitions.h>

TEST(BallbotRaisimConversions, AngularVelocities) {
  const Eigen::Vector3d zeroEulerAngles(Eigen::Vector3d::Zero());

  Eigen::Vector3d omega_base_inWorld = Eigen::Vector3d::Zero();
  auto eulerAnglesDerivative = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(zeroEulerAngles, omega_base_inWorld);
  EXPECT_TRUE(eulerAnglesDerivative.isZero());

  omega_base_inWorld = 0.1 * Eigen::Vector3d::UnitZ();
  eulerAnglesDerivative = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(zeroEulerAngles, omega_base_inWorld);
  Eigen::Vector3d expected = 0.1 * Eigen::Vector3d::UnitX();  // pure yaw rate
  EXPECT_TRUE(eulerAnglesDerivative.isApprox(expected));

  omega_base_inWorld = 0.1 * Eigen::Vector3d::UnitX();
  eulerAnglesDerivative = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(zeroEulerAngles, omega_base_inWorld);
  expected = 0.1 * Eigen::Vector3d::UnitZ();  // pure roll rate
  EXPECT_TRUE(eulerAnglesDerivative.isApprox(expected));

  omega_base_inWorld = 0.1 * Eigen::Vector3d::UnitY();
  eulerAnglesDerivative = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(zeroEulerAngles, omega_base_inWorld);
  expected = 0.1 * Eigen::Vector3d::UnitY();  // pure pitch rate
  EXPECT_TRUE(eulerAnglesDerivative.isApprox(expected));
}

TEST(BallbotRaisimConversions, AngularVelocitiesSelfConsistency) {
  for (int i = 0; i < 10; i++) {
    Eigen::Vector3d eulerAngles = Eigen::Vector3d::Random();
    Eigen::Vector3d omega_base_inWorld = Eigen::Vector3d::Random();

    auto eulerAnglesDerivative = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(eulerAngles, omega_base_inWorld);
    auto omega_base_inWorld_calc = ocs2::eulerAngleZyxDerivativesToAngularVelocityInWorld<double>(eulerAngles, eulerAnglesDerivative);
    auto eulerAnglesDerivative_calc = ocs2::angularVelocityInWorldToEulerAngleZyxDerivatives<double>(eulerAngles, omega_base_inWorld_calc);

    EXPECT_TRUE(eulerAnglesDerivative.isApprox(eulerAnglesDerivative_calc));
    EXPECT_TRUE(omega_base_inWorld.isApprox(omega_base_inWorld_calc));
  }
}

TEST(BallbotRaisimConversions, StateConversionSelfConsistency) {
  ocs2::ballbot::BallbotRaisimConversions conversions;

  for (int i = 0; i < 100; i++) {
    ocs2::vector_t ocs2_state = 0.2 * ocs2::vector_t::Random(ocs2::ballbot::STATE_DIM);
    ocs2::vector_t dummyInput = ocs2::vector_t::Random(ocs2::ballbot::INPUT_DIM);

    // consistency test ocs2 -> raisim -> ocs2
    Eigen::VectorXd q, dq;
    std::tie(q, dq) = conversions.stateToRaisimGenCoordGenVel(ocs2_state, dummyInput);
    auto ocs2_state_calc = conversions.raisimGenCoordGenVelToState(q, dq);
    EXPECT_TRUE(ocs2_state.isApprox(ocs2_state_calc));

    // consistency test raisim -> ocs2 -> raisim
    Eigen::VectorXd q_calc, dq_calc;
    std::tie(q_calc, dq_calc) = conversions.stateToRaisimGenCoordGenVel(ocs2_state_calc, dummyInput);
    EXPECT_TRUE(q.isApprox(q_calc));
    EXPECT_TRUE(dq.isApprox(dq_calc));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
