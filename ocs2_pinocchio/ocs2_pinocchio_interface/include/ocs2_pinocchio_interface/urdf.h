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

#include <string>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <urdf_model/model.h>

namespace ocs2 {

/** Factory function from URDF file
 * @param [in] urdfFile: Path to the URDF file.
 */
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile);

/** Factory function from URDF file with root joint
 * @param [in] urdfFile: Path to the URDF file.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 */
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile, const PinocchioInterface::JointModel& rootJoint);

/** Factory function from URDF string
 * @param [in] xmlString: A URDF loaded as an std::string.
 */
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString);

/** Factory function from URDF string with root joint
 * @param [in] xmlString: A URDF loaded as an std::string.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 */
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString, const PinocchioInterface::JointModel& rootJoint);

/** Factory function from URDF model tree
 * @param [in] urdfTree: Pointer to a URDF model tree.
 */
PinocchioInterface getPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree);

/** Factory function from URDF model tree with root joint
 * @param [in] urdfTree: Pointer to a URDF model tree.
 * @param [in] rootJoint: Root joint to which the robot is attached (eg. a 6 DOF base).
 */
PinocchioInterface getPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree,
                                                      const PinocchioInterface::JointModel& rootJoint);
}  // namespace ocs2
