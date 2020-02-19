#include <gtest/gtest.h>
#include <ocs2_ballbot_raisim_example/BallbotRaisimConversions.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>

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
    ocs2::ballbot::BallbotRaisimConversions::state_vector_t ocs2_state;
    ocs2::ballbot::BallbotRaisimConversions::input_vector_t dummyInput;
    ocs2_state.setRandom();
    ocs2_state *= 0.2;
    dummyInput.setRandom();

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
