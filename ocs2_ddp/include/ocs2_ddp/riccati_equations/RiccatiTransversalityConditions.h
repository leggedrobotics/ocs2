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

#include <Eigen/Dense>
#include <tuple>

#include <ocs2_core/Types.h>
#include <ocs2_core/model_data/ModelData.h>

namespace ocs2 {

static inline std::tuple<matrix_t, vector_t, scalar_t> riccatiTransversalityConditions(const ModelData& jumpModelData, const matrix_t& Sm,
                                                                                       const vector_t& Sv, scalar_t s) {
  // Sm
  const matrix_t SmTransAm = Sm.transpose() * jumpModelData.dynamics.dfdx;
  matrix_t SmPreEvent = jumpModelData.cost.dfdxx;
  SmPreEvent.noalias() += SmTransAm.transpose() * jumpModelData.dynamics.dfdx;

  // Sv
  const vector_t SmHv = Sm * jumpModelData.dynamicsBias;
  vector_t SvPreEvent = jumpModelData.cost.dfdx;
  SvPreEvent.noalias() += jumpModelData.dynamics.dfdx.transpose() * (Sv + SmHv);

  // s
  const scalar_t sPreEvent = s + jumpModelData.cost.f + jumpModelData.dynamicsBias.dot(Sv + 0.5 * SmHv);

  return {SmPreEvent, SvPreEvent, sPreEvent};
}

}  // namespace ocs2
