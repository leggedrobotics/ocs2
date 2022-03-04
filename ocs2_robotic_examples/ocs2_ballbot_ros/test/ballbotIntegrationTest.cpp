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

#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>

#include <ocs2_ballbot/BallbotInterface.h>
#include <ocs2_ballbot/definitions.h>

using namespace ocs2;

TEST(BallbotIntegrationTest, createDummyMRT) {
  const std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/mpc/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  ballbot::BallbotInterface ballbotInterface(taskFile, libFolder);

  MRT_ROS_Interface mrt("ballbot");

  // Dummy ballbot
  MRT_ROS_Dummy_Loop dummyBallbot(mrt, ballbotInterface.mpcSettings().mrtDesiredFrequency_,
                                  ballbotInterface.mpcSettings().mpcDesiredFrequency_);

  // Initialize dummy
  SystemObservation initObservation;
  initObservation.state = ballbotInterface.getInitialState();
  initObservation.input.setZero(ocs2::ballbot::INPUT_DIM);
  initObservation.time = 0.0;
}

TEST(BallbotIntegrationTest, createMPC) {
  const std::string taskFile = ros::package::getPath("ocs2_ballbot") + "/config/mpc/task.info";
  const std::string libFolder = ros::package::getPath("ocs2_ballbot") + "/auto_generated";
  ballbot::BallbotInterface ballbotInterface(taskFile, libFolder);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(ballbotInterface.mpcSettings(), ballbotInterface.ddpSettings(), ballbotInterface.getRollout(),
                               ballbotInterface.getOptimalControlProblem(), ballbotInterface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(ballbotInterface.getReferenceManagerPtr());

  // Create MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, "ballbot");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
