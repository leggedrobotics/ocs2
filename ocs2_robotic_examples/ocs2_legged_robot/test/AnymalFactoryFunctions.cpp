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

#include <ros/package.h>

#include "ocs2_legged_robot/test/AnymalFactoryFunctions.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

namespace {
const std::string ROBOT_URDF_PATH = ros::package::getPath("anymal_c_simple_description") + "/urdf/" + "anymal.urdf";
const std::string ROBOT_TASK_FILE_PATH = ros::package::getPath("ocs2_legged_robot") + "/config/mpc/" + "task.info";
const std::string ROBOT_COMMAND_PATH = ros::package::getPath("ocs2_legged_robot") + "/config/command/" + "targetTrajectories.info";
}  // unnamed namespace

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<PinocchioInterface> createAnymalPinocchioInterface() {
  return std::unique_ptr<PinocchioInterface>(new PinocchioInterface(centroidal_model::createPinocchioInterface(ROBOT_URDF_PATH)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelInfo createAnymalCentroidalModelInfo(const PinocchioInterface& pinocchioInterface, CentroidalModelType centroidalType) {
  const ModelSettings modelSettings;  // default constructor just to get contactNames3DoF
  return centroidal_model::createCentroidalModelInfo(pinocchioInterface, centroidalType,
                                                     centroidal_model::loadDefaultJointState(12, ROBOT_COMMAND_PATH),
                                                     modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<SwitchedModelReferenceManager> createReferenceManager(size_t numFeet) {
  const auto initModeSchedule = loadModeSchedule(ROBOT_TASK_FILE_PATH, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(ROBOT_TASK_FILE_PATH, "defaultModeSequenceTemplate", false);

  const ModelSettings modelSettings;
  std::shared_ptr<GaitSchedule> gaitSchedule(
      new GaitSchedule(initModeSchedule, defaultModeSequenceTemplate, modelSettings.phaseTransitionStanceTime));
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(ROBOT_TASK_FILE_PATH, "swing_trajectory_config", false), numFeet));
  return std::make_shared<SwitchedModelReferenceManager>(gaitSchedule, std::move(swingTrajectoryPlanner));
}

}  // namespace legged_robot
}  // namespace ocs2
