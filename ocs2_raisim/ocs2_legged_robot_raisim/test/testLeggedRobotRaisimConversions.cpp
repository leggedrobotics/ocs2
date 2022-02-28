/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <ocs2_raisim_core/RaisimRolloutSettings.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h"

TEST(LeggedRobotRaisim, Conversions) {
  // paths to files
  std::string taskFile = ros::package::getPath("ocs2_legged_robot") + "/config/mpc/task.info";
  std::string urdfFile = ros::package::getPath("ocs2_robotic_assets") + "/resources/anymal_c/urdf/anymal.urdf";
  std::string referenceFile = ros::package::getPath("ocs2_legged_robot") + "/config/command/reference.info";
  std::string raisimFile = ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info";
  // interface
  ocs2::legged_robot::LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);
  // raisim conversions
  ocs2::RaisimRolloutSettings raisimRolloutSettings(raisimFile, "rollout");
  ocs2::legged_robot::LeggedRobotRaisimConversions conversions(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
                                                               interface.getInitialState());
  // consistency test ocs2 -> raisim -> ocs2
  for (size_t i = 0; i < 100; i++) {
    ocs2::vector_t stateIn(24);
    stateIn.setRandom();
    ocs2::vector_t inputIn(24);
    inputIn.setRandom();

    Eigen::VectorXd q, dq;
    std::tie(q, dq) = conversions.stateToRaisimGenCoordGenVel(stateIn, inputIn);

    ocs2::vector_t stateOut = conversions.raisimGenCoordGenVelToState(q, dq);

    EXPECT_TRUE(stateIn.isApprox(stateOut));
  }
  // consistency test raisim -> ocs2 -> raisim
  for (size_t i = 0; i < 100; i++) {
    Eigen::VectorXd qIn;
    qIn.setRandom(19);
    qIn.segment<4>(3).normalize();

    Eigen::VectorXd dqIn;
    dqIn.setRandom(18);

    ocs2::vector_t state = conversions.raisimGenCoordGenVelToState(qIn, dqIn);
    ocs2::vector_t input = ocs2::vector_t::Zero(24);
    input.tail<12>() = conversions.raisimJointOrderToOcs2JointOrder(dqIn.tail<12>());

    Eigen::VectorXd qOut, dqOut;
    std::tie(qOut, dqOut) = conversions.stateToRaisimGenCoordGenVel(state, input);

    // flip quaternion sign for comparison
    if (qIn(3) * qOut(3) < 0.0) {
      qOut.segment<4>(3) *= -1.0;
    }

    EXPECT_TRUE(qIn.isApprox(qOut));
    EXPECT_TRUE(dqIn.isApprox(dqOut));
  }
}
