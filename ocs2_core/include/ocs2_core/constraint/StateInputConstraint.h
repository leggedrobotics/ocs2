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

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/ConstraintOrder.h>

namespace ocs2 {

/** State-input constraint function base class */
class StateInputConstraint {
 public:
  explicit StateInputConstraint(ConstraintOrder order) : order_(order) {}
  virtual ~StateInputConstraint() = default;
  virtual StateInputConstraint* clone() const = 0;

  /** Get the constraint order (Linear or Quadratic) */
  constexpr ConstraintOrder getOrder() const { return order_; };

  /** Check constraint activity */
  virtual bool isActive(scalar_t time) const { return true; }

  /** Get the size of the constraint vector at given time */
  virtual size_t getNumConstraints(scalar_t time) const = 0;

  /** Get the constraint vector value */
  virtual vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const = 0;

  /** Get the constraint linear approximation */
  virtual VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                   const PreComputation& preComp) const {
    if (order_ == ConstraintOrder::Linear) {
      throw std::runtime_error("[StateInputConstraint] Linear approximation not implemented!");
    } else {
      throw std::runtime_error("[StateInputConstraint] The class only provides Quadratic approximation! call getQuadraticApproximation()");
    }
  }

  /** Get the constraint quadratic approximation */
  virtual VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const PreComputation& preComp) const {
    if (order_ == ConstraintOrder::Quadratic) {
      throw std::runtime_error("[StateConstraint] Quadratic approximation not implemented!");
    } else {
      throw std::runtime_error("[StateConstraint] The class only provides Linear approximation! call getLinearApproximation()");
    }
  }

 protected:
  StateInputConstraint(const StateInputConstraint& rhs) = default;

 private:
  ConstraintOrder order_;
};

// Template for conditional compilation using SFINAE
template <typename T>
using EnableIfStateInputConstraint_t = typename std::enable_if<std::is_same<T, StateInputConstraint>::value, bool>::type;

}  // namespace ocs2
