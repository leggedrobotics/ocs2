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

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include <ocs2_core/cost/StateCostCollection.h>
#include <ocs2_core/cost/StateInputCostCollection.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace ocs2 {

/** Optimal Control Problem definition */
struct OptimalControlProblem final {
  /** System dynamics pointer */
  std::unique_ptr<SystemDynamicsBase> dynamics;

  /* Constraints */
  /** Intermediate equality constraints */
  StateInputConstraintCollection equalityConstraint;
  /** Intermediate state-only equality constraints */
  StateConstraintCollection stateEqualityConstraint;
  /** Intermediate inequality constraints */
  StateInputConstraintCollection inequalityConstraint;
  /** pre-jump constraints */
  StateConstraintCollection preJumpEqualityConstraint;
  /** final constraints */
  StateConstraintCollection finalEqualityConstraint;

  /* Soft constraints */
  /** Intermediate soft constraint penalty cost */
  StateInputCostCollection softConstraint;
  /** Pre-jump soft constraint penalty cost */
  StateCostCollection preJumpSoftConstraint;
  /** Final soft constraint penalty cost */
  StateCostCollection finalSoftConstraint;

  /* Cost */
  /** Intermediate cost */
  StateInputCostCollection cost;
  /** Intermediate state-only cost */
  StateCostCollection stateCost;
  /** Pre-jump cost */
  StateCostCollection preJumpCost;
  /** Final cost */
  StateCostCollection finalCost;

  /** Desired trajectory reference */
  const CostDesiredTrajectories* costDesiredTrajectories = nullptr;  // TODO(mspieler) remove this

  /** The pre-computation module */
  std::unique_ptr<PreComputation> preComputation;

  /** Default constructor */
  OptimalControlProblem();

  /** Copy constructor */
  OptimalControlProblem(const OptimalControlProblem& other);

  /** Move constructor */
  OptimalControlProblem(OptimalControlProblem&& other) noexcept;

  /** Copy assignment */
  OptimalControlProblem& operator=(const OptimalControlProblem& rhs);

  /** Move assignment */
  OptimalControlProblem& operator=(OptimalControlProblem&& rhs);
};

}  // namespace ocs2
