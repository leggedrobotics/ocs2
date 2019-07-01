#include <gtest/gtest.h>

#include <ocs2_ballbot_example/ros_comm/MRT_Network_Interface.h>

//TEST(Ballbot, MRTNetworkInterface) {

 // using namespace ocs2;

//  using state_vector_t = ballbot::MRT_Network_Interface::state_vector_t;
//  using input_vector_t = ballbot::MRT_Network_Interface::input_vector_t;

//  ballbot::MRT_Network_Interface network_interface;

//  ballbot::MRT_Network_Interface::cost_desired_trajectories_t costDesiredTraj;
//  network_interface.resetMpcNode(costDesiredTraj);

//  state_vector_t nextState;
//  input_vector_t nextInput;
//  size_t subsystem;
//  network_interface.rolloutPolicy(0.0, state_vector_t::Zero(), 1.0, nextState, nextInput, subsystem);

//}

int main(int argc, char** argv) {
  ocs2::ballbot::MRT_Network_Interface network_interface;
  return 0;
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
}
