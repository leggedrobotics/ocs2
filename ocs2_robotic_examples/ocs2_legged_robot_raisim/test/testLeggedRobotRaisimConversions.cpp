#include <gtest/gtest.h>

#include <ros/package.h>

#include <ocs2_raisim/RaisimRolloutSettings.h>

#include <ocs2_legged_robot/LeggedRobotInterface.h>

#include "ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h"

TEST(LeggedRobotRaisim, Conversions) {
  // path to config files
  std::string taskFileFolderName = "mpc";
  std::string targetCommandFile = ros::package::getPath("ocs2_legged_robot") + "/config/command/targetTrajectories.info";
  // path to urdf file
  std::string urdfFile = ros::package::getPath("anymal_c_simple_description") + "/urdf/anymal.urdf";
  // interface
  ocs2::legged_robot::LeggedRobotInterface interface(taskFileFolderName, targetCommandFile, urdf::parseURDFFile(urdfFile));
  // raisim conversions
  ocs2::RaisimRolloutSettings raisimRolloutSettings(ros::package::getPath("ocs2_legged_robot_raisim") + "/config/raisim.info", "rollout");
  ocs2::legged_robot::LeggedRobotRaisimConversions conversions(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
                                                               interface.modelSettings());
  // consistency test ocs2 -> raisim -> ocs2
  for (size_t i = 0; i < 100; i++) {
    ocs2::vector_t stateIn(24);
    stateIn.setRandom();
    ocs2::vector_t inputIn(24);
    inputIn.setRandom();

    Eigen::VectorXd q, dq;
    std::tie(q, dq) = conversions.stateToRaisimGenCoordGenVel(stateIn, inputIn);

    const ocs2::vector_t stateOut = conversions.raisimGenCoordGenVelToState(q, dq);

    bool test = stateIn.isApprox(stateOut);
    EXPECT_TRUE(test);
  }
  // consistency test raisim -> ocs2 -> raisim
  for (size_t i = 0; i < 100; i++) {
    Eigen::VectorXd qIn;
    qIn.setRandom(19);
    qIn.segment<4>(3).normalize();

    Eigen::VectorXd dqIn;
    dqIn.setRandom(18);

    ocs2::vector_t state = conversions.raisimGenCoordGenVelToState(qIn, dqIn);
    ocs2::vector_t input(24);
    input.head<12>().setZero();           // contact forces
    input.tail<12>() = dqIn.tail<12>();  // joint velocities

    Eigen::VectorXd qOut, dqOut;
    std::tie(qOut, dqOut) = conversions.stateToRaisimGenCoordGenVel(state, input);

    // flip quaternion sign for comparison
    if (qIn(3) * qOut(3) < 0.0) {
      qOut.segment<4>(3) *= -1.0;
    }

    bool test = qIn.isApprox(qOut);
    EXPECT_TRUE(test);
    test = dqIn.isApprox(dqOut);
    EXPECT_TRUE(test);
  }
}
