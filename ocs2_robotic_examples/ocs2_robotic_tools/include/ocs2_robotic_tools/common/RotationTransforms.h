/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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
#include <array>
#include <cmath>

namespace ocs2 {

/**
 * Compute the quaternion distance measure
 *
 * @param [in] eeQuaternion: measured end effector quaternion.
 * @param [in] commandQuaternion: desired end effector quaternion.
 * @return A 3x1 vector representing the quaternion distance.
 * In particular, if Qd and Qe are the desired and the measured end-effector quaternions,
 * the measured and desired frames are aligned if this vector is 0.
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quaternionDistance(const Eigen::Quaternion<SCALAR_T>& eeQuaternion,
                                                 const Eigen::Quaternion<SCALAR_T>& commandQuaternion) {
    return eeQuaternion.w() * commandQuaternion.vec() - commandQuaternion.w() * eeQuaternion.vec() -
           commandQuaternion.vec().cross(eeQuaternion.vec());
}

/**
 * Compute the quaternion corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding quaternion
 */
template <typename SCALAR_T>
Eigen::Quaternion<SCALAR_T> getQuaternionFromEulerAnglesZyx(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAnglesZyx) {
    SCALAR_T yaw = eulerAnglesZyx(0);
    SCALAR_T pitch = eulerAnglesZyx(1);
    SCALAR_T roll = eulerAnglesZyx(2);
    // Abbreviations for the trigonometric functions
    SCALAR_T cy = cos(yaw * 0.5);
    SCALAR_T sy = sin(yaw * 0.5);
    SCALAR_T cp = cos(pitch * 0.5);
    SCALAR_T sp = sin(pitch * 0.5);
    SCALAR_T cr = cos(roll * 0.5);
    SCALAR_T sr = sin(roll * 0.5);

    Eigen::Quaternion<SCALAR_T> quaternion;
    quaternion.w() = cy * cp * cr + sy * sp * sr;
    quaternion.x() = cy * cp * sr - sy * sp * cr;
    quaternion.y() = sy * cp * sr + cy * sp * cr;
    quaternion.z() = sy * cp * cr - cy * sp * sr;

    return quaternion;
}

}