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

#include <algorithm>
#include <string>
#include <vector>

#include "ocs2_core/Types.h"
#include "ocs2_core/constraint/StateConstraintCollection.h"
#include "ocs2_core/constraint/StateInputConstraintCollection.h"
#include "ocs2_core/misc/LinearInterpolation.h"

namespace ocs2 {

/** The penalty and Lagrangian multipliers associated to a constraint vector of size Multiplier::lagrangian.size() */
struct Multiplier {
  Multiplier() : Multiplier(0.0, vector_t()) {}
  Multiplier(scalar_t penaltyArg, vector_t lagrangianArg) : penalty(penaltyArg), lagrangian(std::move(lagrangianArg)) {}

  scalar_t penalty;
  vector_t lagrangian;
};

/** A const reference view to Multiplier. This is useful for having an array of references to Multipliers. */
struct MultiplierConstRef {
  MultiplierConstRef(const Multiplier& multiplierArg) : penalty(multiplierArg.penalty), lagrangian(multiplierArg.lagrangian) {}
  operator Multiplier() const { return {penalty, lagrangian}; }

  const scalar_t& penalty;
  const vector_t& lagrangian;
};

/**
 * The collection of Multiplier structure for all possible constraint terms in a particular time point.
 * stateEq : An array of state equality constraint terms.
 * stateIneq : An array of state inequality constraint terms.
 * stateInputEq : An array of state-input equality constraint terms.
 * stateInputIneq : An array of state-input inequality constraint terms.
 */
struct MultiplierCollection {
  // state equality
  std::vector<Multiplier> stateEq;
  // state inequality
  std::vector<Multiplier> stateIneq;
  // state-input equality
  std::vector<Multiplier> stateInputEq;
  // state-input inequality
  std::vector<Multiplier> stateInputIneq;

  /** Exchanges the values of MultiplierCollection. */
  void swap(MultiplierCollection& other) {
    stateEq.swap(other.stateEq);
    stateIneq.swap(other.stateIneq);
    stateInputEq.swap(other.stateInputEq);
    stateInputIneq.swap(other.stateInputIneq);
  }

  /** Clears the values of the MultiplierCollection. */
  void clear() {
    stateEq.clear();
    stateIneq.clear();
    stateInputEq.clear();
    stateInputIneq.clear();
  }

  /** Whether the MultiplierCollection is empty. */
  bool empty() const { return stateEq.empty() && stateIneq.empty() && stateInputEq.empty() && stateInputIneq.empty(); }
};

/**
 * Serializes an array of Multiplier structures associated to an array of constraint terms.
 *
 * @ param [in] termsMultiplier : Multipliers associated to an array of constraint terms.
 * @return Serialized vector of the format : (..., termsMultiplier[i].penalty, termsMultiplier[i].lagrangian, ...).
 */
vector_t toVector(const std::vector<Multiplier>& termsMultiplier);

/**
 * Gets the size of constraint terms.
 *
 * @ param [in] termsMultiplier : Multipliers associated to an array of constraint terms.
 * @return An array of constraint terms size. It has the same size as the input array.
 */
size_array_t getSizes(const std::vector<Multiplier>& termsMultiplier);

/**
 * Deserializes the vector to an array of Multiplier structures based on size of constraint terms.
 *
 * @param [in] termsSize : An array of constraint terms size. It as the same size as the output array.
 * @param [in] vec : Serialized array of Multiplier structures of the format :
 *                   (..., termsMultiplier[i].penalty, termsMultiplier[i].lagrangian, ...)
 * @return An array of Multiplier structures associated to an array of constraint terms
 */
std::vector<Multiplier> toMultipliers(const size_array_t& termsSize, const vector_t& vec);

}  // namespace ocs2

namespace ocs2 {
namespace LinearInterpolation {

/**
 * Linearly interpolates a trajectory of Multipliers.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MultiplierConstRef.
 * @return The interpolated Multiplier at indexAlpha.
 */
Multiplier interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierConstRef>& dataArray);

/**
 * Linearly interpolates a trajectory of MultiplierCollections.
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair.
 * @param [in] dataArray : A trajectory of MultiplierCollections.
 * @return The interpolated MultiplierCollection at indexAlpha.
 */
MultiplierCollection interpolate(const index_alpha_t& indexAlpha, const std::vector<MultiplierCollection>& dataArray);

}  // namespace LinearInterpolation
}  // namespace ocs2
