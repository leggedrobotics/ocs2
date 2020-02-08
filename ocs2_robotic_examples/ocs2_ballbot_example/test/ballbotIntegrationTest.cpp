

#include <gtest/gtest.h>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "ocs2_ballbot_example/BallbotInterface.h"
#include "ocs2_ballbot_example/definitions.h"
#include "ocs2_ballbot_example/ros_comm/MRT_ROS_Dummy_Ballbot.h"
#include "ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h"

using namespace ocs2;

TEST(BallbotIntegrationTest, createDummyMRT) {
  std::string taskFileFolderName = "mpc";
  ballbot::BallbotInterface ballbotInterface(taskFileFolderName);

  MRT_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> mrt("ballbot");

  // Dummy ballbot
  ballbot::MRT_ROS_Dummy_Ballbot dummyBallbot(mrt, ballbotInterface.mpcSettings().mrtDesiredFrequency_,
                                              ballbotInterface.mpcSettings().mpcDesiredFrequency_);

  // Initialize dummy
  ballbot::MRT_ROS_Dummy_Ballbot::system_observation_t initObservation;
  initObservation.state() = ballbotInterface.getInitialState();

  ASSERT_TRUE(true);
}

TEST(BallbotIntegrationTest, createMPC) {
  std::string taskFileFolderName = "mpc";
  ballbot::BallbotInterface ballbotInterface(taskFileFolderName);

  // Launch MPC ROS node
  auto mpcPtr = ballbotInterface.getMpc();
  MPC_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> mpcNode(*mpcPtr, "ballbot");

  ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

