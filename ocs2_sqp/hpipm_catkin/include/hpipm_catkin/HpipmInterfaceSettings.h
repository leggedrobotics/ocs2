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

extern "C" {
#include <hpipm_common.h>
}

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace hpipm_interface {

/**
 * Settings forwarded to HPIPM
 */
struct Settings {
  hpipm_mode hpipmMode = hpipm_mode::SPEED;
  int iter_max = 30;
  scalar_t alpha_min = 1e-12;
  scalar_t mu0 = 1e1;
  scalar_t tol_stat = 1e-6;  // res_g_max
  scalar_t tol_eq = 1e-8;    // res_b_max
  scalar_t tol_ineq = 1e-8;  // res_d_max
  scalar_t tol_comp = 1e-8;  // res_m_max
  scalar_t reg_prim = 1e-12;
  int warm_start = 0;
  int pred_corr = 1;
  int ric_alg = 0;  // square root ricatti recursion
};

std::ostream& operator<<(std::ostream& stream, const Settings& settings);

}  // namespace hpipm_interface
}  // namespace ocs2