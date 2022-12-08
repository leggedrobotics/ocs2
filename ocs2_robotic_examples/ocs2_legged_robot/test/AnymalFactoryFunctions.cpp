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

#include <string>

#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_robotic_assets/package_path.h>

#include "ocs2_legged_robot/common/ModelSettings.h"
#include "ocs2_legged_robot/package_path.h"

namespace {
const std::string URDF_FILE = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";
const std::string TASK_FILE = ocs2::legged_robot::getPath() + "/config/mpc/" + "task.info";
const std::string REFERENCE_FILE = ocs2::legged_robot::getPath() + "/config/command/" + "reference.info";
}  // unnamed namespace

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<PinocchioInterface> createAnymalPinocchioInterface() {
  return std::make_unique<PinocchioInterface>(centroidal_model::createPinocchioInterface(URDF_FILE));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelInfo createAnymalCentroidalModelInfo(const PinocchioInterface& pinocchioInterface, CentroidalModelType centroidalType) {
  const ModelSettings modelSettings;  // default constructor just to get contactNames3DoF
  return centroidal_model::createCentroidalModelInfo(pinocchioInterface, centroidalType,
                                                     centroidal_model::loadDefaultJointState(12, REFERENCE_FILE),
                                                     modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<SwitchedModelReferenceManager> createReferenceManager(size_t numFeet) {
  const auto initModeSchedule = loadModeSchedule(REFERENCE_FILE, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(REFERENCE_FILE, "defaultModeSequenceTemplate", false);

  const ModelSettings modelSettings;
  auto gaitSchedule =
      std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings.phaseTransitionStanceTime);
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(TASK_FILE, "swing_trajectory_config", false), numFeet);
  return std::make_shared<SwitchedModelReferenceManager>(std::move(gaitSchedule), std::move(swingTrajectoryPlanner));
}

}  // namespace legged_robot
}  // namespace ocs2
