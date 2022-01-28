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

#pragma once

#include <string>
#include <vector>

#include <urdf_parser/urdf_parser.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_centroidal_model/CentroidalModelInfo.h"

namespace ocs2 {
namespace centroidal_model {

/**
 * Create a CentroidalModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 */
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath);

/**
 * Create a CentroidalModel PinocchioInterface from a URDF.
 * @param [in] urdfFilePath: The absolute path to the URDF file for the robot.
 * @param [in] jointNames: Any joint that is not listed in jointNames (a.k.a the extraneous joints) will be removed from the urdf.
 */
PinocchioInterface createPinocchioInterface(const std::string& urdfFilePath, const std::vector<std::string>& jointNames);

/**
 * Create a scalar-typed CentroidalModelInfo.
 * @param [in] interface: Pinocchio interface
 * @param [in] type: Type of template model (SRBD or FRBD)
 * @param [in] nominalJointAngles: nominal joint angles used in the SRBD model.
 * @param [in] threeDofContactNames: Names of end-effectors with 3 DoF contacts (force)
 * @param [in] sixDofContactNames: Names of end-effectors with 6 DoF contacts (force + torque)
 * @return CentroidalModelInfo
 */
CentroidalModelInfo createCentroidalModelInfo(const PinocchioInterface& interface, const CentroidalModelType& type,
                                              const vector_t& nominalJointAngles, const std::vector<std::string>& threeDofContactNames,
                                              const std::vector<std::string>& sixDofContactNames);

/** Load CentroidalModelType for a config file */
CentroidalModelType loadCentroidalType(const std::string& configFilePath, const std::string& fieldName = "centroidalModelType");

/** Load default joint state for a config file */
vector_t loadDefaultJointState(size_t numJointState, const std::string& configFilePath, const std::string& fieldName = "defaultJointState");

}  // namespace centroidal_model
}  // namespace ocs2
