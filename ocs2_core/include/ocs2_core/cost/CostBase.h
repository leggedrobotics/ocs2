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

#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>

namespace ocs2 {

/**
 * Cost Function base class.
 */
class CostBase {
 public:
  /** Constructor */
  explicit CostBase(PreComputation* preComp) = default;

  /** Default destructor */
  virtual ~CostBase() = default;

  /** Sets the desired state and input trajectories used in the cost function. */
  virtual void setCostDesiredTrajectoriesPtr(const CostDesiredTrajectories* costDesiredTrajectoriesPtr);

  /** Clone */
  virtual CostBase* clone(PreComputation* preComp) const = 0;

  /** Evaluate the cost */
  scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u);
  /** Evaluate the pre-jump cost */
  scalar_t getPreJumpValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation* preComp);
  /** Evaluate the final cost */
  scalar_t getFinalValue(scalar_t t, const vector_t& x, const PreComputation* preComp);

  /** Evaluate the cost quadratic approximation */
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u);
  /** Evaluate the pre-jump cost quadratic approximation */
  ScalarFunctionQuadraticApproximation getPreJumpQuadraticApproximation(scalar_t t, const vector_t& x);
  /** Evaluate the final cost quadratic approximation */
  ScalarFunctionQuadraticApproximation getFinalQuadraticApproximation(scalar_t t, const vector_t& x);

 protected:
  /** Copy constructor */
  CostBase(const CostBase& other) = default;

  PreComputation* preCompPtr_ = nullptr;
  const CostDesiredTrajectories* costDesiredTrajectoriesPtr_ = nullptr;

  CostCollection<StateInputCost> intermediateCosts_;
  CostCollection<StateCost> intermediateStateCosts_;
  CostCollection<StateCost> finalCosts_;
  CostCollection<StateCost> preJumpCosts_;
};

}  // namespace ocs2
