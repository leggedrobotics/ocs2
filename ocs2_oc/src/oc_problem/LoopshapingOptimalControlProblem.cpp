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

#include "ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h"

#include <ocs2_core/loopshaping/Loopshaping.h>

namespace ocs2 {
namespace LoopshapingOptimalControlProblem {

OptimalControlProblem create(const OptimalControlProblem& problem, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  OptimalControlProblem augmentedProblem;

  // Dynamics
  augmentedProblem.dynamicsPtr = LoopshapingDynamics::create(*problem.dynamicsPtr, loopshapingDefinition);

  // Equality constraints
  augmentedProblem.equalityConstraintPtr = LoopshapingConstraint::create(*problem.equalityConstraintPtr, loopshapingDefinition);
  augmentedProblem.stateEqualityConstraintPtr = LoopshapingConstraint::create(*problem.stateEqualityConstraintPtr, loopshapingDefinition);
  augmentedProblem.preJumpEqualityConstraintPtr =
      LoopshapingConstraint::create(*problem.preJumpEqualityConstraintPtr, loopshapingDefinition);
  augmentedProblem.finalEqualityConstraintPtr = LoopshapingConstraint::create(*problem.finalEqualityConstraintPtr, loopshapingDefinition);

  // Lagrangians
  augmentedProblem.equalityLagrangianPtr = LoopshapingSoftConstraint::create(*problem.equalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.stateEqualityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.stateEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.inequalityLagrangianPtr = LoopshapingSoftConstraint::create(*problem.inequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.stateInequalityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.stateInequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.preJumpEqualityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.preJumpEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.preJumpInequalityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.preJumpInequalityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.finalEqualityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.finalEqualityLagrangianPtr, loopshapingDefinition);
  augmentedProblem.finalInequalityLagrangianPtr =
      LoopshapingSoftConstraint::create(*problem.finalInequalityLagrangianPtr, loopshapingDefinition);

  // Soft constraints
  augmentedProblem.softConstraintPtr = LoopshapingSoftConstraint::create(*problem.softConstraintPtr, loopshapingDefinition);
  augmentedProblem.stateSoftConstraintPtr = LoopshapingSoftConstraint::create(*problem.stateSoftConstraintPtr, loopshapingDefinition);
  augmentedProblem.preJumpSoftConstraintPtr = LoopshapingSoftConstraint::create(*problem.preJumpSoftConstraintPtr, loopshapingDefinition);
  augmentedProblem.finalSoftConstraintPtr = LoopshapingSoftConstraint::create(*problem.finalSoftConstraintPtr, loopshapingDefinition);

  // Cost
  augmentedProblem.costPtr = LoopshapingCost::create(*problem.costPtr, loopshapingDefinition);
  augmentedProblem.stateCostPtr = LoopshapingCost::create(*problem.stateCostPtr, loopshapingDefinition);
  augmentedProblem.preJumpCostPtr = LoopshapingCost::create(*problem.preJumpCostPtr, loopshapingDefinition);
  augmentedProblem.finalCostPtr = LoopshapingCost::create(*problem.finalCostPtr, loopshapingDefinition);

  // Pre-computation
  augmentedProblem.preComputationPtr.reset(new LoopshapingPreComputation(*problem.preComputationPtr, loopshapingDefinition));

  /** The cost desired trajectories (will be substitute by ReferenceManager) */
  augmentedProblem.targetTrajectoriesPtr = problem.targetTrajectoriesPtr;

  return augmentedProblem;
}

}  // namespace LoopshapingOptimalControlProblem
}  // namespace ocs2
