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

#include <ocs2_core/Types.h>
#include <ocs2_robotic_assets/package_path.h>

namespace ocs2 {
namespace centroidal_model {

enum anymal : size_t {
  STATE_DIM = 6 + 6 + 12,  // centroidal momentum, generalized coordinates
  INPUT_DIM = 4 * 3 + 12,  // end effector forces, joint configuration
};

static const std::vector<std::string> anymal3DofContactNames = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
static const std::vector<std::string> anymal6DofContactNames = {};
static const std::string anymalUrdfFile = ocs2::robotic_assets::getPath() + "/resources/anymal_c/urdf/anymal.urdf";

inline ocs2::vector_t getInitialState() {
  ocs2::vector_t x0 = ocs2::vector_t::Zero(anymal::STATE_DIM);

  x0(0) = 0.0;  // vcom_x
  x0(1) = 0.0;  // vcom_y
  x0(2) = 0.0;  // vcom_z
  x0(3) = 0.0;  // L_x / mass
  x0(4) = 0.0;  // L_y / mass
  x0(5) = 0.0;  // L_z / mass

  // Base Pose: [position, orientation]
  x0(6) = 0.0;   // p_base_x
  x0(7) = 0.0;   // p_base_y
  x0(8) = 0.53;  // p_base_z
  x0(9) = 0.0;   // theta_base_z
  x0(10) = 0.0;  // theta_base_y
  x0(11) = 0.0;  // theta_base_x

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

  return x0;
}

}  // namespace centroidal_model
}  // namespace ocs2
