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

#include <ocs2_core/ComputationRequest.h>
#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Pre-Computation base class.
 *
 * This is an optional module for sharing computation between system dynamics, cost and constraint
 * terms. The virtual request callback are called before getting the value or approximation.
 * The callbacks take a set of requested computation items, which must be computed and stored
 * in the PreComputation object. The same PreComputation is passed to the getter methods of
 * dynamics, cost and constraint terms, which can make use of the shared pre-computation.
 *
 * If pre-computation is not used, a default constructed PreComputation() can be passed to the getters.
 */
class PreComputation {
 public:
  /** Constructor */
  PreComputation() = default;

  /** Default destructor */
  virtual ~PreComputation() = default;

  /** Clone */
  virtual PreComputation* clone() const { return new PreComputation(*this); }

  /** Request callback */
  virtual void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {}

  /** Request callback at jump event time */
  virtual void requestPreJump(RequestSet request, scalar_t t, const vector_t& x) {}

  /** Request callback at final time */
  virtual void requestFinal(RequestSet request, scalar_t t, const vector_t& x) {}

 protected:
  /** Copy constructor */
  PreComputation(const PreComputation& other) = default;
};

/** Helper to cast to const reference of derived class. */
template <typename Derived>
const Derived& cast(const PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  assert(dynamic_cast<const Derived*>(&preComputation) != nullptr);
  return static_cast<const Derived&>(preComputation);
}

/** Helper to cast to reference of derived class. */
template <typename Derived>
Derived& cast(PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  assert(dynamic_cast<Derived*>(&preComputation) != nullptr);
  return static_cast<Derived&>(preComputation);
}

}  // namespace ocs2