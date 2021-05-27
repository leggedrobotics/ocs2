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

#include <ocs2_core/Types.h>

namespace ocs2 {

/**
 * Pre-Computation base class.
 *
 * This is an optional module for sharing computation between system dynamics, cost and constraint
 * terms. The virtual request callback must be called before getting the value or approximation.
 * The callbacks take a set of requested computation items, which must be computed and stored
 * in the PreComputation object. The same PreComputation is passed to the getter mehtods of
 * dynamics, cost and constraint terms, which can make use the shared pre-computation.
 *
 * If pre-computation is not used, a default constructed PreComputation() can be passed to the getters.
 */
class PreComputation {
 public:
  /**
   * Request of a set of Computation items.
   * It can be composed by union of two Requests using operator+().
   */
  struct Request {
    enum Computation { Dynamics = 1, Cost = 2, Constraint = 4, SoftConstraint = 8, Approximation = 16 };

    /** Constructor, implicit construction is allowed */
    constexpr Request(Computation computationFlag) : flags(computationFlag) {}

    /** Test if this Request contains item */
    constexpr bool contains(Computation item) const { return (static_cast<int>(flags) & static_cast<int>(item)) != 0; }

    /** Test if this Request contains any Computation of other (non-empty intersection) */
    constexpr bool containsAny(Request other) const { return (static_cast<int>(flags) & static_cast<int>(other.flags)) != 0; }

    /** Test if this Request contains all Computations of other (subset) */
    constexpr bool containsAll(Request other) const {
      return (static_cast<int>(flags) & static_cast<int>(other.flags)) == static_cast<int>(other.flags);
    }

    /** flags bitfield */
    Computation flags;
  };

  /** Constructor */
  PreComputation() = default;

  /** Default destructor */
  virtual ~PreComputation() = default;

  /** Clone */
  virtual PreComputation* clone() const { return new PreComputation(*this); }

  /** Request callback */
  virtual void request(Request request, scalar_t t, const vector_t& x, const vector_t& u) {}

  /** Request callback at jump event time */
  virtual void requestPreJump(Request request, scalar_t t, const vector_t& x) {}

  /** Request callback at final time */
  virtual void requestFinal(Request request, scalar_t t, const vector_t& x) {}

 protected:
  /** Copy constructor */
  PreComputation(const PreComputation& other) = default;
};

/* Helper to cast to const reference of derived class. */
template <typename Derived>
const Derived& cast(const PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  assert(dynamic_cast<const Derived*>(&preComputation) != nullptr);
  return static_cast<const Derived&>(preComputation);
}

/* Helper to cast to reference of derived class. */
template <typename Derived>
Derived& cast(PreComputation& preComputation) {
  static_assert(std::is_base_of<PreComputation, Derived>::value, "Template argument must derive from PreComputation");
  assert(dynamic_cast<Derived*>(&preComputation) != nullptr);
  return static_cast<Derived&>(preComputation);
}

/** Union of two Request sets */
inline constexpr PreComputation::Request operator+(PreComputation::Request a, PreComputation::Request b) {
  return PreComputation::Request(static_cast<PreComputation::Request::Computation>(static_cast<int>(a.flags) | static_cast<int>(b.flags)));
}

/** Get a Request out of two Computation requests */
inline constexpr PreComputation::Request operator+(PreComputation::Request::Computation a, PreComputation::Request::Computation b) {
  return PreComputation::Request(a) + PreComputation::Request(b);
}

}  // namespace ocs2
