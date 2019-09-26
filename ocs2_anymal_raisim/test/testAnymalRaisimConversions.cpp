#include <gtest/gtest.h>
#include <ros/package.h>
#include <ocs2_anymal_raisim/AnymalRaisimConversions.h>

TEST(AnymalRaisim, Conversions) {
  constexpr size_t JOINT_COORD_SIZE = 12;
  constexpr size_t STATE_DIM = 24;
  constexpr size_t INPUT_DIM = 24;

  using state_vector_t = Eigen::Matrix<double, STATE_DIM, 1>;
  using input_vector_t = Eigen::Matrix<double, INPUT_DIM, 1>;

  std::shared_ptr<anymal::OCS2AnymalInterface> anymalInterface(
      new anymal::OCS2AnymalInterface(ros::package::getPath("ocs2_anymal_interface") + "/config/mpc"));

  using mrt_t = switched_model::MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  std::shared_ptr<mrt_t> anymalMrt(new mrt_t(anymalInterface, "anymal"));

  auto conversions = anymal::AnymalRaisimConversions(anymalMrt, anymalInterface);

  // consistency test ocs2 -> raisim -> ocs2
  for (int i = 0; i < 100; i++) {
    state_vector_t ocs2_state_in;
    ocs2_state_in.setRandom();
    input_vector_t ocs2_input_in;
    ocs2_input_in.setRandom();

    Eigen::VectorXd raisim_q, raisim_dq;
    std::tie(raisim_q, raisim_dq) = conversions.stateToRaisimGenCoordGenVel(ocs2_state_in, ocs2_input_in);

    state_vector_t ocs2_state_out = conversions.raisimGenCoordGenVelToState(raisim_q, raisim_dq);

    bool test = ocs2_state_in.isApprox(ocs2_state_out);
    EXPECT_TRUE(test);
  }

  std::cout << "-----------------------------------------------------------------\n"
            << "-----------------------------------------------------------------" << std::endl;

  // consistency test raisim -> ocs2 -> raisim
  for (int i = 0; i < 100; i++) {
    Eigen::VectorXd raisim_q_in;
    raisim_q_in.setRandom(19);
    raisim_q_in.segment<4>(3).normalize();

    Eigen::VectorXd raisim_dq_in;
    raisim_dq_in.setRandom(18);

    state_vector_t ocs2_state = conversions.raisimGenCoordGenVelToState(raisim_q_in, raisim_dq_in);
    input_vector_t ocs2_input;
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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
