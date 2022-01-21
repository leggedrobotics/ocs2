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

#include <gtest/gtest.h>

#include <ocs2_core/augmented_lagrangian/Multiplier.h>
#include <ocs2_core/misc/LinearInterpolation.h>

namespace ocs2 {
/**
 * Linearly interpolates a trajectory of MultiplierCollections.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MultiplierCollections.
 * @return The interpolated MultiplierCollection at indexAlpha.
 */
inline MultiplierCollection interpolateNew(const LinearInterpolation::index_alpha_t& indexAlpha, const std::vector<MultiplierCollection>& dataArray) {
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    const int index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;
    bool areSameSize = true;

    const auto lhs_stateEq = toVector(dataArray[index].stateEq);
    const auto rhs_stateEq = toVector(dataArray[index + 1].stateEq);
    areSameSize = areSameSize && (lhs_stateEq.size() == rhs_stateEq.size());

    const auto lhs_stateIneq = toVector(dataArray[index].stateIneq);
    const auto rhs_stateIneq = toVector(dataArray[index + 1].stateIneq);
    areSameSize = areSameSize && (lhs_stateIneq.size() == rhs_stateIneq.size());

    const auto lhs_stateInputEq = toVector(dataArray[index].stateInputEq);
    const auto rhs_stateInputEq = toVector(dataArray[index + 1].stateInputEq);
    areSameSize = areSameSize && (lhs_stateInputEq.size() == rhs_stateInputEq.size());

    const auto lhs_stateInputIneq = toVector(dataArray[index].stateInputIneq);
    const auto rhs_stateInputIneq = toVector(dataArray[index + 1].stateInputIneq);
    areSameSize = areSameSize && (lhs_stateInputIneq.size() == rhs_stateInputIneq.size());

    if (areSameSize) {
      const auto f = [alpha](const vector_t& lhs, const vector_t& rhs) -> vector_t { return alpha * lhs + (scalar_t(1.0) - alpha) * rhs; };
      MultiplierCollection out;
      out.stateEq = toMultipliers(getSizes(dataArray[index].stateEq), f(lhs_stateEq, rhs_stateEq));
      out.stateIneq = toMultipliers(getSizes(dataArray[index].stateIneq), f(lhs_stateIneq, rhs_stateIneq));
      out.stateInputEq = toMultipliers(getSizes(dataArray[index].stateInputEq), f(lhs_stateInputEq, rhs_stateInputEq));
      out.stateInputIneq = toMultipliers(getSizes(dataArray[index].stateInputIneq), f(lhs_stateInputIneq, rhs_stateInputIneq));
      return out;

    } else {
      return (alpha > 0.5) ? dataArray[index] : dataArray[index + 1];
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    return dataArray[0];
  }
}
}  // namespace ocs2
