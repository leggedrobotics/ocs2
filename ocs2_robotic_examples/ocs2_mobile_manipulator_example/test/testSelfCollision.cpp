/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>
#include <ocs2_self_collision/SelfCollision.h>
#include <ocs2_self_collision/SelfCollisionCppAd.h>

const std::string urdfPath = ros::package::getPath("ocs2_mobile_manipulator_example") + "/urdf/mobile_manipulator.urdf";
const std::string libraryFolder = ros::package::getPath("ocs2_mobile_manipulator_example") + "/auto_generated";
const ocs2::scalar_t minDistance = 0.1;

// initial joint configuration
const ocs2::vector_t jointPositon = (ocs2::vector_t(9) << 1.0, 1.0, 0.5, 2.5, -1.0, 1.5, 0.0, 1.0, 0.0).finished();

TEST(testSelfCollision, AnalyticalVsAutoDiffValue) {
  ocs2::PinocchioInterface pinocchioInterface(mobile_manipulator::MobileManipulatorInterface::buildPinocchioInterface(urdfPath));
  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath, pinocchioInterface, {{1, 4}, {1, 6}, {1, 9}});

  ocs2::SelfCollision selfCollision(minDistance);
  ocs2::SelfCollisionCppAd selfCollisionCppAd(minDistance);
  selfCollisionCppAd.initialize(pinocchioInterface, geometryInterface, "testSelfCollision", libraryFolder, true, false);

  pinocchioInterface.forwardKinematics(jointPositon);

  const auto dist1 = selfCollision.getValue(pinocchioInterface, geometryInterface);
  const auto dist2 = selfCollisionCppAd.getValue(pinocchioInterface, geometryInterface);
  EXPECT_TRUE(dist1.isApprox(dist2));
}

TEST(testSelfCollision, AnalyticalVsAutoDiffApproximation) {
  ocs2::PinocchioInterface pinocchioInterface(mobile_manipulator::MobileManipulatorInterface::buildPinocchioInterface(urdfPath));
  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath, pinocchioInterface, {{1, 4}, {1, 6}, {1, 9}});

  ocs2::SelfCollision selfCollision(minDistance);
  ocs2::SelfCollisionCppAd selfCollisionCppAd(minDistance);
  selfCollisionCppAd.initialize(pinocchioInterface, geometryInterface, "testSelfCollision", libraryFolder, true, false);

  pinocchioInterface.computeJointJacobians(jointPositon);  // also computes forwardKinematics
  pinocchioInterface.updateGlobalPlacements();

  ocs2::vector_t d1, d2;
  ocs2::matrix_t Jd1, Jd2;

  std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface, geometryInterface);
  std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, geometryInterface, jointPositon);
  EXPECT_TRUE(d1.isApprox(d2));
  EXPECT_TRUE(Jd1.isApprox(Jd2));
}

TEST(testSelfCollision, AnalyticalValueAndApproximation) {
  ocs2::PinocchioInterface pinocchioInterface(mobile_manipulator::MobileManipulatorInterface::buildPinocchioInterface(urdfPath));
  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath, pinocchioInterface, {{1, 4}, {1, 6}, {1, 9}});

  ocs2::SelfCollision selfCollision(minDistance);
  pinocchioInterface.computeJointJacobians(jointPositon);  // also computes forwardKinematics
  pinocchioInterface.updateGlobalPlacements();

  const auto d1 = selfCollision.getLinearApproximation(pinocchioInterface, geometryInterface).first;
  const auto d2 = selfCollision.getValue(pinocchioInterface, geometryInterface);
  EXPECT_TRUE(d1.isApprox(d2));
}

TEST(testSelfCollision, testRandomJointPositions) {
  ocs2::PinocchioInterface pinocchioInterface(mobile_manipulator::MobileManipulatorInterface::buildPinocchioInterface(urdfPath));
  ocs2::PinocchioGeometryInterface geometryInterface(urdfPath, pinocchioInterface, {{1, 4}, {1, 6}, {1, 9}});

  ocs2::SelfCollision selfCollision(minDistance);
  ocs2::SelfCollisionCppAd selfCollisionCppAd(minDistance);
  selfCollisionCppAd.initialize(pinocchioInterface, geometryInterface, "testSelfCollision", libraryFolder, true, false);

  for (int i = 0; i < 10; i++) {
    ocs2::vector_t q = ocs2::vector_t::Random(9);
    pinocchioInterface.computeJointJacobians(q);  // also computes forwardKinematics
    pinocchioInterface.updateGlobalPlacements();

    ocs2::vector_t d1, d2;
    ocs2::matrix_t Jd1, Jd2;

    std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface, geometryInterface);
    std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, geometryInterface, q);

    if (!d1.isApprox(d2)) {
      std::cerr << "[d1]: " << d1.transpose() << '\n';
      std::cerr << "[d2]: " << d2.transpose() << '\n';
    }
    if (!Jd1.isApprox(Jd2)) {
      std::cerr << "[Jd1]:\n" << Jd1 << '\n';
      std::cerr << "[Jd2]:\n" << Jd2 << '\n';
    }

    ASSERT_TRUE(d1.isApprox(d2));
    ASSERT_TRUE(Jd1.isApprox(Jd2));
  }
}
