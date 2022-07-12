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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_assets/package_path.h>
#include <ocs2_self_collision/SelfCollision.h>
#include <ocs2_self_collision/SelfCollisionCppAd.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/MobileManipulatorInterface.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

class TestSelfCollision : public ::testing::Test {
 public:
  TestSelfCollision()
      : pinocchioInterface(createMobileManipulatorPinocchioInterface()), geometryInterface(pinocchioInterface, collisionPairs) {}

  void computeValue(PinocchioInterface& pinocchioInterface, const vector_t q) {
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();
    pinocchio::forwardKinematics(model, data, q);
  }

  void computeLinearApproximation(PinocchioInterface& pinocchioInterface, const vector_t q) {
    const auto& model = pinocchioInterface.getModel();
    auto& data = pinocchioInterface.getData();
    pinocchio::computeJointJacobians(model, data, q);  // also computes forwardKinematics
    pinocchio::updateGlobalPlacements(model, data);
  }

  // initial joint configuration
  const vector_t jointPositon = (vector_t(9) << 1.0, 1.0, 0.5, 2.5, -1.0, 1.5, 0.0, 1.0, 0.0).finished();
  const std::vector<std::pair<size_t, size_t>> collisionPairs = {{1, 4}, {1, 6}, {1, 9}};

  const std::string libraryFolder = ocs2::mobile_manipulator::getPath() + "/auto_generated";
  const scalar_t minDistance = 0.1;

  PinocchioInterface pinocchioInterface;
  PinocchioGeometryInterface geometryInterface;

 protected:
  PinocchioInterface createMobileManipulatorPinocchioInterface() {
    const std::string urdfPath = ocs2::robotic_assets::getPath() + "/resources/mobile_manipulator/mabi_mobile/urdf/mabi_mobile.urdf";
    const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/mabi_mobile/task.info";

    // read manipulator type
    ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    // read the joints to make fixed
    std::vector<std::string> removeJointNames;
    loadData::loadStdVector<std::string>(taskFile, "model_information.removeJoints", removeJointNames, false);
    // initialize pinocchio interface
    return createPinocchioInterface(urdfPath, modelType, removeJointNames);
  }
};

TEST_F(TestSelfCollision, AnalyticalVsAutoDiffValue) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", libraryFolder, true,
                                        false);

  computeValue(pinocchioInterface, jointPositon);

  const auto dist1 = selfCollision.getValue(pinocchioInterface);
  const auto dist2 = selfCollisionCppAd.getValue(pinocchioInterface);
  EXPECT_TRUE(dist1.isApprox(dist2));
}

TEST_F(TestSelfCollision, AnalyticalVsAutoDiffApproximation) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", libraryFolder, true,
                                        false);

  computeLinearApproximation(pinocchioInterface, jointPositon);

  vector_t d1, d2;
  matrix_t Jd1, Jd2;

  std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface);
  std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, jointPositon);
  EXPECT_TRUE(d1.isApprox(d2));
  EXPECT_TRUE(Jd1.isApprox(Jd2));
}

TEST_F(TestSelfCollision, AnalyticalValueAndApproximation) {
  SelfCollision selfCollision(geometryInterface, minDistance);

  computeLinearApproximation(pinocchioInterface, jointPositon);

  const auto d1 = selfCollision.getLinearApproximation(pinocchioInterface).first;
  const auto d2 = selfCollision.getValue(pinocchioInterface);
  EXPECT_TRUE(d1.isApprox(d2));
}

TEST_F(TestSelfCollision, testRandomJointPositions) {
  SelfCollision selfCollision(geometryInterface, minDistance);
  SelfCollisionCppAd selfCollisionCppAd(pinocchioInterface, geometryInterface, minDistance, "testSelfCollision", libraryFolder, true,
                                        false);

  for (int i = 0; i < 10; i++) {
    vector_t q = vector_t::Random(9);
    computeLinearApproximation(pinocchioInterface, q);

    vector_t d1, d2;
    matrix_t Jd1, Jd2;

    std::tie(d1, Jd1) = selfCollision.getLinearApproximation(pinocchioInterface);
    std::tie(d2, Jd2) = selfCollisionCppAd.getLinearApproximation(pinocchioInterface, q);

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
