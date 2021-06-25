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

#include <ocs2_legged_robot_example/common/definitions.h>

#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelInfoTpl<scalar_t> createCentroidalModelInfo(CentroidalModelType type, const std::string& robotUrdfPath,
                                                           const std::string& robotCommanPath,
                                                           const std::vector<std::string>& contactNames3Dof,
                                                           const std::vector<std::string>& contactNames6Dof) {
  // Add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  PinocchioInterface pinocchioInterface = getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);

  constexpr size_t numActuatedDof = 12;
  vector_t initJoints(numActuatedDof);
  ocs2::loadData::loadEigenMatrix(robotCommanPath, "defaultJointState", initJoints);
  vector_t qNominal(6 + numActuatedDof);
  qNominal << vector_t::Zero(6), initJoints;

  return CentroidalModelInfoTpl<scalar_t>(pinocchioInterface, type, qNominal, contactNames3Dof, contactNames6Dof);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelInfoTpl<ad_scalar_t> createCentroidalModelInfoAd(CentroidalModelType type, const std::string& robotUrdfPath,
                                                                const std::string& robotCommanPath,
                                                                const std::vector<std::string>& contactNames3Dof,
                                                                const std::vector<std::string>& contactNames6Dof) {
  // Add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());
  PinocchioInterface pinocchioInterface = getPinocchioInterfaceFromUrdfFile(robotUrdfPath, jointComposite);

  constexpr size_t numActuatedDof = 12;
  vector_t initJoints(numActuatedDof);
  ocs2::loadData::loadEigenMatrix(robotCommanPath, "defaultJointState", initJoints);
  vector_t qNominal(6 + numActuatedDof);
  qNominal << vector_t::Zero(6), initJoints;

  return CentroidalModelInfoTpl<ad_scalar_t>(pinocchioInterface.toCppAd(), type, qNominal.cast<ad_scalar_t>(), contactNames3Dof,
                                             contactNames6Dof);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalModelType loadCentroidalType(const std::string& taskFilePath) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFilePath, pt);
  const size_t type = pt.template get<size_t>("centroidalModelType");
  return static_cast<CentroidalModelType>(type);
}

}  // namespace legged_robot
}  // namespace ocs2
