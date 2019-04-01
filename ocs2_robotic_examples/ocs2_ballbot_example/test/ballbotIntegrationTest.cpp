//
// Created by rgrandia on 29.03.19.
//

#include <gtest/gtest.h>

#include "ocs2_ballbot_example/BallbotInterface.h"
#include "ocs2_ballbot_example/definitions.h"
#include "ocs2_ballbot_example/ros_comm/MPC_ROS_Ballbot.h"
#include "ocs2_ballbot_example/ros_comm/MRT_ROS_Ballbot.h"
#include "ocs2_ballbot_example/ros_comm/MRT_ROS_Dummy_Ballbot.h"

using namespace ocs2;
using namespace ballbot;

TEST(BallbotIntegrationTest, createDummyMRT) {
  std::string taskFileFolderName = "mpc";
  BallbotInterface ballbotInterface(taskFileFolderName);

  typedef MRT_ROS_Ballbot mrt_t;
  typedef mrt_t::BASE::Ptr mrt_base_ptr_t;
  typedef mrt_t::scalar_t scalar_t;
  typedef mrt_t::system_observation_t system_observation_t;

  mrt_base_ptr_t mrtPtr(new mrt_t(
      !ballbotInterface.mpcSettings().useFeedbackPolicy_,
      "ballbot"));

  // Dummy ballbot
  MRT_ROS_Dummy_Ballbot dummyBallbot(
      mrtPtr,
      ballbotInterface.mpcSettings().mrtDesiredFrequency_,
      ballbotInterface.mpcSettings().mpcDesiredFrequency_);

  // Initialize dummy
  MRT_ROS_Dummy_Ballbot::system_observation_t initObservation;
  ballbotInterface.getInitialState(initObservation.state());
  dummyBallbot.init(initObservation);

  ASSERT_TRUE(true);
}

TEST(BallbotIntegrationTest, createMPC) {
  std::string taskFileFolderName = "mpc";
  BallbotInterface ballbotInterface(taskFileFolderName);

  // Launch MPC ROS node
  MPC_ROS_Ballbot mpcNode(*ballbotInterface.getMPCPtr(), "ballbot");

  ASSERT_TRUE(true);
}



int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

