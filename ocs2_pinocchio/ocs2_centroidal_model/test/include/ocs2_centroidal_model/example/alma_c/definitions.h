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

#include <cstddef>
#include <string>
#include <vector>

#include <ros/package.h>

#include <ocs2_core/Types.h>

namespace ocs2 {

enum alma_c : size_t {
  DOF_PER_ARM_NUM = 4,
  ARM_CONTACTS_NUM = 1,   // each contact involves a 3-DOF force or a 6-DOF wrench
  ARM_CONTACTS_DOF = 3,   // either 3 or 6
  FOOT_CONTACTS_NUM = 4,  // each contact involves a 3-DOF force
  TOTAL_CONTACTS_DIM = 3 * FOOT_CONTACTS_NUM + ARM_CONTACTS_DOF * ARM_CONTACTS_NUM,

  BASE_DOF_NUM = 6,
  ACTUATED_DOF_NUM = 3 * FOOT_CONTACTS_NUM + DOF_PER_ARM_NUM,
  GENERALIZED_VEL_NUM = BASE_DOF_NUM + ACTUATED_DOF_NUM,

  STATE_DIM = 2 * BASE_DOF_NUM + ACTUATED_DOF_NUM,
  INPUT_DIM = TOTAL_CONTACTS_DIM + ACTUATED_DOF_NUM,
};

static const std::vector<std::string> alma3DofContactNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT", "dynaarm_END_EFFECTOR"};
static const std::vector<std::string> alma6DofContactNames = {};
static const std::string almaUrdfPath =
    ros::package::getPath("ocs2_centroidal_model") + "/test/include/ocs2_centroidal_model/example/alma_c/alma_c.urdf";
static const std::string almaCppAdModelPath = ros::package::getPath("ocs2_centroidal_model") + "/test/CppAD_generated";

static const ocs2::vector_t almaInitialState = []() {
  ocs2::vector_t x0 = ocs2::vector_t(size_t(alma_c::STATE_DIM));

  x0(0) = 0.0;  // vcom_x
  x0(1) = 0.0;  // vcom_y
  x0(2) = 0.0;  // vcom_z
  x0(3) = 0.0;  // L_x / mass
  x0(4) = 0.0;  // L_y / mass
  x0(5) = 0.0;  // L_z / mass

  // Base Pose: [position, orientation]
  x0(6) = 0.0;    // p_base_x
  x0(7) = 0.0;    // p_base_y
  x0(8) = 0.527;  // p_base_z
  x0(9) = 0.0;    // theta_base_z
  x0(10) = 0.0;   // theta_base_y
  x0(11) = 0.0;   // theta_base_x

  // Leg Joint Positions: [LF, LH, RF, RH]
  x0(12) = -0.10;  // LF_HAA
  x0(13) = 0.7;    // LF_HFE
  x0(14) = -1.0;   // LF_KFE
  x0(15) = -0.10;  // LH_HAA
  x0(16) = -0.7;   // LH_HFE
  x0(17) = 1.0;    // LH_KFE
  x0(18) = 0.10;   // RF_HAA
  x0(19) = 0.7;    // RF_HFE
  x0(20) = -1.0;   // RF_KFE
  x0(21) = 0.10;   // RH_HAA
  x0(22) = -0.7;   // RH_HFE
  x0(23) = 1.0;    // RH_KFE

  // Arm Joint Positions
  x0(24) = 0.0;  // SH_ROT
  x0(25) = 2.3;  // SH_FLE
  x0(26) = 2.3;  // EL_FLE
  x0(27) = 0.0;  // FA_ROT

  return x0;
}();

}  // namespace ocs2
