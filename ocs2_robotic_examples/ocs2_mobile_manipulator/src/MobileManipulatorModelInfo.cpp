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

#include "ocs2_mobile_manipulator/MobileManipulatorModelInfo.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace mobile_manipulator {

template <>
template <>
MobileManipulatorModelInfoCppAd MobileManipulatorModelInfo::toCppAd() const {
  MobileManipulatorModelInfoCppAd cppAdInfo;

  cppAdInfo.manipulatorModelType = this->manipulatorModelType;
  cppAdInfo.stateDim = this->stateDim;
  cppAdInfo.inputDim = this->inputDim;
  cppAdInfo.armDim = this->armDim;
  cppAdInfo.baseFrame = this->baseFrame;
  cppAdInfo.eeFrame = this->eeFrame;
  cppAdInfo.dofNames = this->dofNames;

  return cppAdInfo;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::VectorXd getArmJointPositions(Eigen::VectorXd state, const MobileManipulatorModelInfo& info) {
  return state.tail(info.armDim);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Vector3d getBasePosition(Eigen::VectorXd state, const MobileManipulatorModelInfo& info) {
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Vector3d::Zero();
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the first three entries correspond to base position
      Eigen::Vector3d position;
      position << state(0), state(1), state(2);
      return position;
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume 2D base position
      Eigen::Vector3d position;
      position << state(0), state(1), 0.0;
      return position;
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Eigen::Quaterniond getBaseOrientation(Eigen::VectorXd state, const MobileManipulatorModelInfo& info) {
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Quaterniond::Identity();
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the base orientation is given by ZYX joints
      return ::ocs2::getQuaternionFromEulerAnglesZyx<double>(state.segment<3>(3));
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume only yaw
      return Eigen::Quaterniond(Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }
}

// explicit template instantiation
template struct ocs2::mobile_manipulator::MobileManipulatorModelInfoTpl<ocs2::scalar_t>;
template struct ocs2::mobile_manipulator::MobileManipulatorModelInfoTpl<ocs2::ad_scalar_t>;

}  // namespace mobile_manipulator
}  // namespace ocs2
