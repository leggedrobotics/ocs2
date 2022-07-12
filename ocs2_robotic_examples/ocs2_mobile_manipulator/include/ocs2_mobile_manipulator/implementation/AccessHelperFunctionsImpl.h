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

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace mobile_manipulator {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Matrix<SCALAR, 3, 1>::Zero();
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the first three entries correspond to base position
      return state.head(3);
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // for floating arm, the first three entries correspond to base position
      return state.head(3);
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume 2D base position
      return Eigen::Matrix<SCALAR, 3, 1>(state(0), state(1), 0.0);
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve the position vector based on robot type.
  switch (info.manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      // for default arm, we assume robot is at identity pose
      return Eigen::Quaternion<SCALAR>::Identity();
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      // for floating arm, the base orientation is given by ZYX joints
      return ::ocs2::getQuaternionFromEulerAnglesZyx<SCALAR>(state.segment(3, 3));
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      // for floating arm, the base orientation is given by ZYX joints
      return ::ocs2::getQuaternionFromEulerAnglesZyx<SCALAR>(state.segment(3, 3));
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      // for wheel-based, we assume only yaw
      return Eigen::Quaternion<SCALAR>(Eigen::AngleAxis<SCALAR>(state(2), Eigen::Matrix<SCALAR, 3, 1>::UnitZ()));
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(const Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info) {
  assert(state.rows() == info.stateDim);
  assert(state.cols() == 1);
  // resolve for arm dof start index
  const size_t startRow = info.stateDim - info.armDim;
  return Eigen::Block<const Derived, -1, 1>(state.derived(), startRow, 0, info.armDim, 1);
}

}  // namespace mobile_manipulator
}  // namespace ocs2
