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

#include <gtest/gtest.h>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_assets/package_path.h>

#include "ocs2_mobile_manipulator/FactoryFunctions.h"
#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPinocchioMapping.h"
#include "ocs2_mobile_manipulator/MobileManipulatorPreComputation.h"
#include "ocs2_mobile_manipulator/constraint/EndEffectorConstraint.h"
#include "ocs2_mobile_manipulator/package_path.h"

using namespace ocs2;
using namespace mobile_manipulator;

class testEndEffectorConstraint : public ::testing::Test {
 public:
  using quaternion_t = EndEffectorConstraint::quaternion_t;
  using vector3_t = EndEffectorConstraint::vector3_t;

  testEndEffectorConstraint()
      : pinocchioInterface(createMobileManipulatorPinocchioInterface()),
        modelInfo(loadManipulatorModelInfo()),
        pinocchioMapping(modelInfo) {
    // initialize reference manager
    const vector_t positionOrientation = (vector_t(7) << vector3_t::Zero(), quaternion_t(1, 0, 0, 0).coeffs()).finished();
    referenceManagerPtr.reset(new ReferenceManager(TargetTrajectories({0.0}, {positionOrientation})));

    // initialize kinematics
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematics(pinocchioInterface, pinocchioMapping, {modelInfo.eeFrame}));
    preComputationPtr.reset(new MobileManipulatorPreComputation(pinocchioInterface, modelInfo));

    x.resize(modelInfo.stateDim);
    x << 1.0, 1.0, 0.5, 2.5, -1.0, 1.5, 0.0, 1.0, 0.0;
  }

  vector_t x;
  PinocchioInterface pinocchioInterface;
  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr;
  std::unique_ptr<MobileManipulatorPreComputation> preComputationPtr;
  std::shared_ptr<ReferenceManager> referenceManagerPtr;
  MobileManipulatorPinocchioMapping pinocchioMapping;
  ManipulatorModelInfo modelInfo;

 protected:
  ManipulatorModelInfo loadManipulatorModelInfo() {
    // files
    const std::string taskFile = ocs2::mobile_manipulator::getPath() + "/config/mabi_mobile/task.info";
    // read the task file
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    // resolve meta-information about the model
    // read manipulator type
    ManipulatorModelType modelType = mobile_manipulator::loadManipulatorType(taskFile, "model_information.manipulatorModelType");
    // read the frame names
    std::string baseFrame, eeFrame;
    loadData::loadPtreeValue<std::string>(pt, baseFrame, "model_information.baseFrame", false);
    loadData::loadPtreeValue<std::string>(pt, eeFrame, "model_information.eeFrame", false);
    // return model
    return mobile_manipulator::createManipulatorModelInfo(pinocchioInterface, modelType, baseFrame, eeFrame);
  }

  PinocchioInterface createMobileManipulatorPinocchioInterface() {
    // files
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

TEST_F(testEndEffectorConstraint, testConstraintEvaluation) {
  EndEffectorConstraint eeConstraint(*eeKinematicsPtr, *referenceManagerPtr);

  auto& pinocchioInterface = preComputationPtr->getPinocchioInterface();
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const auto q = pinocchioMapping.getPinocchioJointPosition(x);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data);

  std::cerr << "constraint:\n" << eeConstraint.getValue(0.0, x, *preComputationPtr) << '\n';
  std::cerr << "approximation:\n" << eeConstraint.getLinearApproximation(0.0, x, *preComputationPtr);
}
