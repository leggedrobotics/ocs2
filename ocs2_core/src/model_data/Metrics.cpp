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

#include "ocs2_core/model_data/Metrics.h"

namespace ocs2 {
namespace LinearInterpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Metrics interpolate(const index_alpha_t& indexAlpha, const std::vector<MetricsConstRef>& dataArray) {
  const auto penalty = interpolate(indexAlpha, dataArray,
                                   [](const std::vector<MetricsConstRef>& array, size_t t) -> const scalar_t& { return array[t].penalty; });

  const auto constraint = interpolate(
      indexAlpha, dataArray, [](const std::vector<MetricsConstRef>& array, size_t t) -> const vector_t& { return array[t].constraint; });

  return {penalty, constraint};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MetricsCollection interpolate(const index_alpha_t& indexAlpha, const std::vector<MetricsCollection>& dataArray) {
  // number of terms
  const auto ind = indexAlpha.second > 0.5 ? indexAlpha.first : indexAlpha.first + 1;
  const size_t mumStateEq = dataArray[ind].stateEqLagrangian.size();
  const size_t mumStateIneq = dataArray[ind].stateIneqLagrangian.size();
  const size_t mumStateInputEq = dataArray[ind].stateInputEqLagrangian.size();
  const size_t mumStateInputIneq = dataArray[ind].stateInputIneqLagrangian.size();

  MetricsCollection out;

  // state equality
  out.stateEqLagrangian.reserve(mumStateEq);
  for (size_t i = 0; i < mumStateEq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateEqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateEqLagrangian[i].constraint;
    });
    out.stateEqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state inequality
  out.stateIneqLagrangian.reserve(mumStateIneq);
  for (size_t i = 0; i < mumStateIneq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateIneqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateIneqLagrangian[i].constraint;
    });
    out.stateIneqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state-input equality
  out.stateInputEqLagrangian.reserve(mumStateInputEq);
  for (size_t i = 0; i < mumStateInputEq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputEqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputEqLagrangian[i].constraint;
    });
    out.stateInputEqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  // state-input inequality
  out.stateInputIneqLagrangian.reserve(mumStateInputIneq);
  for (size_t i = 0; i < mumStateInputIneq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputIneqLagrangian[i].penalty;
    });
    auto constraint = interpolate(indexAlpha, dataArray, [i](const std::vector<MetricsCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputIneqLagrangian[i].constraint;
    });
    out.stateInputIneqLagrangian.emplace_back(penalty, std::move(constraint));
  }  // end of i loop

  return out;
}

}  // namespace LinearInterpolation
}  // namespace ocs2
