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

#include <Eigen/Core>

#include "ocs2_mobile_manipulator/ManipulatorModelInfo.h"

namespace ocs2 {
namespace mobile_manipulator {

/**
 * Provides read/write access to the base position.
 */
template <typename SCALAR>
Eigen::Matrix<SCALAR, 3, 1> getBasePosition(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info);

/**
 * Provides read/write access to the base orientation.
 */
template <typename SCALAR>
Eigen::Quaternion<SCALAR> getBaseOrientation(const Eigen::Matrix<SCALAR, -1, 1>& state, const ManipulatorModelInfo& info);

/**
 * Provides read/write access to the arm joint angles.
 */
template <typename Derived>
Eigen::Block<Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info);

/**
 * Provides read access to the arm joint angles.
 */
template <typename Derived>
const Eigen::Block<const Derived, -1, 1> getArmJointAngles(Eigen::MatrixBase<Derived>& state, const ManipulatorModelInfo& info);

}  // namespace mobile_manipulator
}  // namespace ocs2

#include "implementation/AccessHelperFunctionsImpl.h"
