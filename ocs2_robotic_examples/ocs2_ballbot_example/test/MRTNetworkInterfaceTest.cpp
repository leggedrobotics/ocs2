#include <gtest/gtest.h>

#include <ocs2_ballbot_example/definitions.h>
#include <ocs2_ballbot_example/dynamics/BallbotSystemDynamics.h>
#include <ocs2_ballbot_example/ros_comm/MRT_Network_Interface.h>

TEST(Ballbot, MRTNetworkInterface) {
  using namespace ocs2;

  using dim_t = Dimensions<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;
  using state_vector_t = dim_t::state_vector_t;
  using input_vector_t = dim_t::input_vector_t;

  std::cout << "creating interface " << __LINE__ << std::endl;
  MRT_Network_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> network_interface(
      "/home/jcarius/Desktop/mpc_policy_learning_data/2019-08-26_ballbot_empty_policy_script.pt");
  std::cout << "creating interface success " << __LINE__ << std::endl;
  ballbot::BallbotSystemDynamics dynamics;
  dynamics.initialize("ballbotDynamics");
  network_interface.initRollout(dynamics, Rollout_Settings());
  std::cout << "init rollout success " << __LINE__ << std::endl;

  state_vector_t nextState;
  input_vector_t nextInput;
  size_t subsystem;

  std::cout << "evaluate policy " << std::endl;
  network_interface.evaluatePolicy(0.0, state_vector_t::Zero(), nextState, nextInput, subsystem);

  std::cout << "rollout policy " << __LINE__ << std::endl;
  network_interface.rolloutPolicy(0.0, state_vector_t::Zero(), 1.0, nextState, nextInput, subsystem);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
