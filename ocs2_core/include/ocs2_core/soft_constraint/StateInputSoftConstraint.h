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

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>

namespace ocs2 {

/**
 *   Implements the cost penalty for state-input constraint terms
 *   \f$ h_i(x, u) \quad \forall  i \in [1,..,M] \f$
 *
 *   penalty(t, x, u) = \f$ \sum_{i=1}^{M} p(t, h_i(x, u)) \f$
 *
 *   The scalar penalty function \f$ p() \f$ and its derivatives are provided by the user.
 *   This class uses the chain rule to compute the second-order approximation of the constraint-penalty. In the case that the
 *   second-order approximation of constraint is not provided, it employs a Gauss-Newton approximation technique which only
 *   relies on the first-order approximation. In general, the penalty function can be a function of time.
 *
 *   A few commonly-used penalty functions have been provided by the toolbox such as Relaxed-Barrier and Squared-Hinge
 *   penalty functions.
 */
class StateInputSoftConstraint final : public StateInputCost {
 public:
  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr, std::vector<std::unique_ptr<PenaltyBase>> penaltyPtrArray);

  /**
   * Constructor.
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyFunction: A pointer to the penalty function on the constraint.
   */
  StateInputSoftConstraint(std::unique_ptr<StateInputConstraint> constraintPtr, std::unique_ptr<PenaltyBase> penaltyFunction);

  ~StateInputSoftConstraint() override = default;

  /** Gets the wrapped constraint. */
  template <typename Derived = StateInputConstraint>
  Derived& get() {
    static_assert(std::is_base_of<StateInputConstraint, Derived>::value, "Template argument must derive from StateInputConstraint");
    return dynamic_cast<Derived&>(*constraintPtr_);
  }

  StateInputSoftConstraint* clone() const override;

  bool isActive(scalar_t time) const override;

  scalar_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& /* targetTrajectories */,
                    const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const TargetTrajectories& /* targetTrajectories */,
                                                                 const PreComputation& preComp) const override;

 private:
  StateInputSoftConstraint(const StateInputSoftConstraint& other);

  std::unique_ptr<StateInputConstraint> constraintPtr_;
  MultidimensionalPenalty penalty_;
};

}  // namespace ocs2
