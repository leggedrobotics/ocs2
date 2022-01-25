#include <gtest/gtest.h>

#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>

#include <ocs2_anymal_models/AnymalModels.h>

TEST(AnymalRaisim, Conversions) {
  for (auto model : {anymal::AnymalModel::Bear, anymal::AnymalModel::Camel}) {
    auto anymalCom = anymal::getAnymalComModel(model);
    auto anymalKinematics = anymal::getAnymalKinematics(model);
    auto anymalWbd = anymal::getWholebodyDynamics(model);
    anymal::AnymalRaisimConversions conversions(*anymalCom, *anymalKinematics, *anymalWbd);

    // consistency test ocs2 -> raisim -> ocs2
    for (int i = 0; i < 100; i++) {
      switched_model::comkino_state_t ocs2_state_in;
      ocs2_state_in.setRandom();
      switched_model::comkino_input_t ocs2_input_in;
      ocs2_input_in.setRandom();

      Eigen::VectorXd raisim_q, raisim_dq;
      std::tie(raisim_q, raisim_dq) = conversions.stateToRaisimGenCoordGenVel(ocs2_state_in, ocs2_input_in);

      const switched_model::comkino_state_t ocs2_state_out = conversions.raisimGenCoordGenVelToState(raisim_q, raisim_dq);

      bool test = ocs2_state_in.isApprox(ocs2_state_out);
      EXPECT_TRUE(test);
    }

    // consistency test raisim -> ocs2 -> raisim
    for (int i = 0; i < 100; i++) {
      Eigen::VectorXd raisim_q_in;
      raisim_q_in.setRandom(19);
      raisim_q_in.segment<4>(3).normalize();

      Eigen::VectorXd raisim_dq_in;
      raisim_dq_in.setRandom(18);

      switched_model::comkino_state_t ocs2_state = conversions.raisimGenCoordGenVelToState(raisim_q_in, raisim_dq_in);
      switched_model::comkino_input_t ocs2_input;
      ocs2_input.head<12>().setZero();                  // contact forces
      ocs2_input.tail<12>() = raisim_dq_in.tail<12>();  // joint velocities

      Eigen::VectorXd raisim_q_out, raisim_dq_out;
      std::tie(raisim_q_out, raisim_dq_out) = conversions.stateToRaisimGenCoordGenVel(ocs2_state, ocs2_input);

      // flip quaternion sign for comparison
      if (raisim_q_in(3) * raisim_q_out(3) < 0.0) {
        raisim_q_out.segment<4>(3) *= -1.0;
      }

      bool test = raisim_q_in.isApprox(raisim_q_out);
      EXPECT_TRUE(test);
      test = raisim_dq_in.isApprox(raisim_dq_out);
      EXPECT_TRUE(test);
    }
  }
}
