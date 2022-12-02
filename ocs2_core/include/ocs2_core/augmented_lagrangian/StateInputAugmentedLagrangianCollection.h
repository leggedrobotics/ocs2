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

#include <functional>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/Collection.h>

#include "ocs2_core/augmented_lagrangian/StateInputAugmentedLagrangianInterface.h"

namespace ocs2 {

/**
 * State-input Augmented Lagrangian penalty class combining a collection of constraint terms.
 *
 * This class collects a variable number of Augmented Lagrangian penalty terms and provides methods to get the
 * summed values and quadratic approximations. Each term can be accessed through its string name.
 */
class StateInputAugmentedLagrangianCollection : public Collection<StateInputAugmentedLagrangianInterface> {
 public:
  StateInputAugmentedLagrangianCollection() = default;
  ~StateInputAugmentedLagrangianCollection() override = default;
  StateInputAugmentedLagrangianCollection* clone() const override;

  /** Get total number of active constraints. */
  size_t getNumberOfActiveConstraints(scalar_t time) const;

  /** Get state constraints and their penalties for each active term */
  virtual std::vector<LagrangianMetrics> getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                                  const std::vector<Multiplier>& termsMultiplier, const PreComputation& preComp) const;

  /** Get the sum of state-input Lagrangian penalties quadratic approximation */
  virtual ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                         const std::vector<Multiplier>& termsMultiplier,
                                                                         const PreComputation& preComp) const;

  /** Update Lagrange/penalty multipliers and the penalty value for each active term. */
  virtual void updateLagrangian(scalar_t time, const vector_t& state, const vector_t& input, std::vector<LagrangianMetrics>& termsMetrics,
                                std::vector<Multiplier>& termsMultiplier) const;

  /** Initialize Lagrange/penalty multipliers for each active term. */
  void initializeLagrangian(scalar_t time, std::vector<Multiplier>& termsMultiplier) const;

 protected:
  StateInputAugmentedLagrangianCollection(const StateInputAugmentedLagrangianCollection& other) = default;
};

}  // namespace ocs2
