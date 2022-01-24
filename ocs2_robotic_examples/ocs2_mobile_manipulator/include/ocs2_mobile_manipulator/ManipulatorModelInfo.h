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

// C/C++
#include <string>
#include <vector>

namespace ocs2 {
namespace mobile_manipulator {

/**
 * @brief Defines various manipulator models.
 */
enum class ManipulatorModelType {
  DefaultManipulator = 0,                   // default model from the parsed URDF directly
  WheelBasedMobileManipulator = 1,          // adds actuatable XY-Yaw joints to the model parsed from URDF
  FloatingArmManipulator = 2,               // adds dummy XYZ-RPY joints to the model parsed from URDF
  FullyActuatedFloatingArmManipulator = 3,  // adds actuatable XYZ-RPY joints to the model parsed from URDF
};

/**
 * @brief A data structure to store manipulator information.
 *
 * The attributes are filled by resolving the URDF model parsed.
 */
struct ManipulatorModelInfo {
  ManipulatorModelType manipulatorModelType;  // type of manipulator: floating-base, fully-actuated floating-base, wheel-base, default
  size_t stateDim;                            // number of states needed to define the system flow map
  size_t inputDim;                            // number of inputs needed to define the system flow map
  size_t armDim;                              // number of DOFs in the robot arm
  std::string baseFrame;                      // name of the root frame of the robot
  std::string eeFrame;                        // name of the end-effector frame of the robot
  std::vector<std::string> dofNames;          // name of the actuated DOFs in the robot
};

/**
 * @brief Returns a string for a ManipulatorModelType for retrieving data from a .info file
 */
static std::string modelTypeEnumToString(ManipulatorModelType manipulatorModelType) {
  std::string manipulatorModelTypeString;

  switch (manipulatorModelType) {
    case ManipulatorModelType::DefaultManipulator: {
      manipulatorModelTypeString = "defaultManipulator";
      break;
    }
    case ManipulatorModelType::FloatingArmManipulator: {
      manipulatorModelTypeString = "floatingArmManipulator";
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      manipulatorModelTypeString = "fullyActuatedFloatingArmManipulator";
      break;
    }
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      manipulatorModelTypeString = "wheelBasedMobileManipulator";
      break;
    }
    default:
      throw std::invalid_argument("Invalid manipulator model type provided.");
      break;
  }

  return manipulatorModelTypeString;
}

}  // namespace mobile_manipulator
}  // namespace ocs2
