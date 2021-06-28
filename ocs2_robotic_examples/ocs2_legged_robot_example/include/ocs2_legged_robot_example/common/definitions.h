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

#pragma once

#include <array>
#include <cstddef>
#include <vector>

// ros
#include <ros/package.h>

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>

/**
 * centroidal model definition:
 *
 * state = [linear_momentum / mass, angular_momentum / mass, base_position, base_orientation_zyx, joint_angles]
 * input = [feet_force, joint_velocities] + slack variables corresponding to the inequality constraints
 * lambda: Force order [LF, RF, LH, RH] in World Frame (3x1)
 * qj: Joint velocities per leg [HAA, HFE, KFE] (3x1)
 */
namespace ocs2 {
namespace legged_robot {

template <typename T>
using feet_array_t = std::array<T, 4>;
using contact_flag_t = feet_array_t<bool>;

const std::string ROBOT_NAME_ = "legged_robot";
const std::string ROBOT_URDF_PATH_ = ros::package::getPath("anymal_c_simple_description") + "/urdf/" + "anymal.urdf";
const std::string ROBOT_COMMAND_PATH_ = ros::package::getPath("ocs2_legged_robot_example") + "/config/command/" + "targetTrajectories.info";
const std::string ROBOT_TASK_FILE_PATH_ = ros::package::getPath("ocs2_legged_robot_example") + "/config/mpc/" + "task.info";

// This is only used to get names for the knees and to check urdf for extra joints that need to be fixed.
const static std::vector<std::string> JOINT_NAMES_{"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                                   "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

const static std::vector<std::string> CONTACT_NAMES_3_DOF_{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
const static std::vector<std::string> CONTACT_NAMES_6_DOF_{};

const auto centroidalModelInfo = centroidal_model::createCentroidalModelInfo(
    centroidal_model::createPinocchioInterface(ROBOT_URDF_PATH_), centroidal_model::loadCentroidalType(ROBOT_TASK_FILE_PATH_),
    centroidal_model::loadDefaultJointState(12, ROBOT_COMMAND_PATH_), CONTACT_NAMES_3_DOF_, CONTACT_NAMES_6_DOF_);

}  // namespace legged_robot
}  // namespace ocs2
