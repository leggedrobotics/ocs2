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

#include <type_traits>

namespace ocs2 {

/** Types of computation that can be simultaneously requested */
enum class Request { Dynamics = 1, Cost = 2, Constraint = 4, SoftConstraint = 8, Approximation = 16 };

/**
 * A set of computation requests
 * It can be composed by union of two Requests using operator+().
 */
class RequestSet {
 public:
  /** Constructor, implicit conversion allowed */
  constexpr RequestSet(Request computationFlag) : flags_(computationFlag) {}  // NOLINT(google-explicit-constructor)

  /** Check if this Request set contains item */
  constexpr bool contains(Request item) const;

  /** Check if this Request set contains any Computation of other (non-empty intersection) */
  constexpr bool containsAny(RequestSet other) const;

  /** Check if this Request set contains all Computations of other (subset) */
  constexpr bool containsAll(RequestSet other) const;

  /** Union of two Request sets */
  friend constexpr RequestSet operator+(RequestSet a, RequestSet b);

 private:
  //! Internal representation of the set through a bitfield
  Request flags_;
};

/** Bitwise | on the underlying type of Request */
constexpr Request operator|(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return static_cast<Request>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}

/** Bitwise & on the underlying type of Request */
constexpr Request operator&(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return static_cast<Request>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}

/** Check if the Bitwise representation of two Request has empty union */
constexpr bool isUnionEmpty(Request lhs, Request rhs) {
  using underlying = typename std::underlying_type<Request>::type;
  return (static_cast<underlying>(lhs) & static_cast<underlying>(rhs)) == underlying(0);
}

/** Create a RequestSet out of two requests */
constexpr RequestSet operator+(Request a, Request b) {
  return a | b;
}

constexpr bool RequestSet::contains(Request item) const {
  return !isUnionEmpty(flags_, item);
}

constexpr bool RequestSet::containsAny(RequestSet other) const {
  return !isUnionEmpty(flags_, other.flags_);
}

constexpr bool RequestSet::containsAll(RequestSet other) const {
  return (flags_ & other.flags_) == other.flags_;
}

constexpr RequestSet operator+(RequestSet a, RequestSet b) {
  return {a.flags_ | b.flags_};
}

}  // namespace ocs2
