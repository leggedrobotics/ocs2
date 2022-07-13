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

#include "ocs2_core/model_data/Multiplier.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t toVector(const std::vector<Multiplier>& termsMultiplier) {
  size_t n = 0;
  std::for_each(termsMultiplier.begin(), termsMultiplier.end(), [&](const Multiplier& m) { n += (1 + m.lagrangian.size()); });

  vector_t vec(n);
  size_t head = 0;
  for (const auto& m : termsMultiplier) {
    vec(head) = m.penalty;
    vec.segment(head + 1, m.lagrangian.size()) = m.lagrangian;
    head += 1 + m.lagrangian.size();
  }  // end of i loop

  return vec;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::vector<Multiplier> toMultipliers(const size_array_t& termsSize, const vector_t& vec) {
  std::vector<Multiplier> multipliers;
  multipliers.reserve(termsSize.size());

  size_t head = 0;
  for (const auto& l : termsSize) {
    multipliers.emplace_back(vec(head), vec.segment(head + 1, l));
    head += 1 + l;
  }  // end of i loop

  return multipliers;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_array_t getSizes(const std::vector<Multiplier>& termsMultiplier) {
  size_array_t s(termsMultiplier.size());
  std::transform(termsMultiplier.begin(), termsMultiplier.end(), s.begin(),
                 [](const Multiplier& m) { return static_cast<size_t>(m.lagrangian.size()); });
  return s;
}

}  // namespace ocs2

namespace ocs2 {
namespace LinearInterpolation {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Multiplier interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierConstRef>& dataArray) {
  const auto penalty = interpolate(
      indexAlpha, dataArray, [](const std::vector<MultiplierConstRef>& array, size_t t) -> const scalar_t& { return array[t].penalty; });

  const auto lagrangian = interpolate(
      indexAlpha, dataArray, [](const std::vector<MultiplierConstRef>& array, size_t t) -> const vector_t& { return array[t].lagrangian; });

  return {penalty, lagrangian};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiplierCollection interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierCollection>& dataArray) {
  // number of terms
  const auto ind = indexAlpha.second > 0.5 ? indexAlpha.first : indexAlpha.first + 1;
  const size_t mumStateEq = dataArray[ind].stateEq.size();
  const size_t mumStateIneq = dataArray[ind].stateIneq.size();
  const size_t mumStateInputEq = dataArray[ind].stateInputEq.size();
  const size_t mumStateInputIneq = dataArray[ind].stateInputIneq.size();

  MultiplierCollection out;

  // state equality
  out.stateEq.reserve(mumStateEq);
  for (size_t i = 0; i < mumStateEq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateEq[i].penalty;
    });
    auto lagrangian = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateEq[i].lagrangian;
    });
    out.stateEq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state inequality
  out.stateIneq.reserve(mumStateIneq);
  for (size_t i = 0; i < mumStateIneq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateIneq[i].penalty;
    });
    auto lagrangian = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateIneq[i].lagrangian;
    });
    out.stateIneq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state-input equality
  out.stateInputEq.reserve(mumStateInputEq);
  for (size_t i = 0; i < mumStateInputEq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputEq[i].penalty;
    });
    auto lagrangian = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputEq[i].lagrangian;
    });
    out.stateInputEq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  // state-input inequality
  out.stateInputIneq.reserve(mumStateInputIneq);
  for (size_t i = 0; i < mumStateInputIneq; i++) {
    auto penalty = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const scalar_t& {
      return array[t].stateInputIneq[i].penalty;
    });
    auto lagrangian = interpolate(indexAlpha, dataArray, [i](const std::vector<MultiplierCollection>& array, size_t t) -> const vector_t& {
      return array[t].stateInputIneq[i].lagrangian;
    });
    out.stateInputIneq.emplace_back(penalty, std::move(lagrangian));
  }  // end of i loop

  return out;
}

}  // namespace LinearInterpolation
}  // namespace ocs2
