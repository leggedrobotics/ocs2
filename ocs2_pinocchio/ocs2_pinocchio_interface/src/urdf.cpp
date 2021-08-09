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

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <urdf_parser/urdf_parser.h>

#include "ocs2_pinocchio_interface/urdf.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree != nullptr) {
    return getPinocchioInterfaceFromUrdfModel(urdfTree);
  } else {
    throw std::invalid_argument("The file " + urdfFile + " does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfFile(const std::string& urdfFile, const PinocchioInterface::JointModel& rootJoint) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFile);
  if (urdfTree != nullptr) {
    return getPinocchioInterfaceFromUrdfModel(urdfTree, rootJoint);
  } else {
    throw std::invalid_argument("The file " + urdfFile + " does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
  if (urdfTree != nullptr) {
    return getPinocchioInterfaceFromUrdfModel(urdfTree);
  } else {
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfString(const std::string& xmlString, const PinocchioInterface::JointModel& rootJoint) {
  ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDF(xmlString);
  if (urdfTree != nullptr) {
    return getPinocchioInterfaceFromUrdfModel(urdfTree, rootJoint);
  } else {
    throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree) {
  pinocchio::ModelTpl<scalar_t> model;
  pinocchio::urdf::buildModel(urdfTree, model);
  return PinocchioInterface(model, urdfTree);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface getPinocchioInterfaceFromUrdfModel(const std::shared_ptr<::urdf::ModelInterface>& urdfTree,
                                                      const PinocchioInterface::JointModel& rootJoint) {
  pinocchio::ModelTpl<scalar_t> model;
  pinocchio::urdf::buildModel(urdfTree, rootJoint, model);
  return PinocchioInterface(model, urdfTree);
}

}  // namespace ocs2
