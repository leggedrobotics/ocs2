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

#include <ocs2_core/augmented_lagrangian/StateInputAugmentedLagrangianInterface.h>
#include <ocs2_core/penalties/MultidimensionalPenalty.h>

namespace ocs2 {

/** The base class for Augmented Lagrangian penalty of state-input constraint. */
class StateInputAugmentedLagrangian final : public StateInputAugmentedLagrangianInterface {
 public:
  /**
   * Constructor.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtrArray: An array of pointers to the penalty function on the constraint.
   */
  StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint> constraintPtr,
                                std::vector<std::unique_ptr<augmented::AugmentedPenaltyBase>> penaltyPtrArray);

  /**
   * Constructor.
   * @note This allows a varying number of constraints and uses the same penalty function for each constraint.
   * @param [in] constraintPtr: A pointer to the constraint which will be enforced as soft constraints.
   * @param [in] penaltyPtr: A pointer to the penalty function on the constraint.
   */
  StateInputAugmentedLagrangian(std::unique_ptr<StateInputConstraint> constraintPtr,
                                std::unique_ptr<augmented::AugmentedPenaltyBase> penaltyPtr);

  StateInputAugmentedLagrangian* clone() const override;
  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override;

  LagrangianMetrics getValue(scalar_t time, const vector_t& state, const vector_t& input, const Multiplier& multiplier,
                             const PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const Multiplier& multiplier,
                                                                 const PreComputation& preComp) const override;

  std::pair<Multiplier, scalar_t> updateLagrangian(scalar_t time, const vector_t& /*state*/, const vector_t& /*input*/,
                                                   const vector_t& constraint, const Multiplier& multiplier) const override;

  Multiplier initializeLagrangian(scalar_t time) const override;

  /** Gets the wrapped constraint. */
  template <typename Derived = StateInputConstraint>
  Derived& get() {
    static_assert(std::is_base_of<StateInputConstraint, Derived>::value, "Template argument must derive from StateInputConstraint!");
    return dynamic_cast<Derived&>(*constraintPtr_);
  }

 private:
  StateInputAugmentedLagrangian(const StateInputAugmentedLagrangian& other);

  std::unique_ptr<StateInputConstraint> constraintPtr_;
  MultidimensionalPenalty penalty_;
};

}  // namespace ocs2
