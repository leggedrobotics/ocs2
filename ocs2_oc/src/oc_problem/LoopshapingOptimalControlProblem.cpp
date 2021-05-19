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

#include <ocs2_core/loopshaping/Loopshaping.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>

namespace ocs2 {
namespace LoopshapingOptimalControlProblem {

std::unique_ptr<OptimalControlProblem> create(const OptimalControlProblem& problem,
                                              std::shared_ptr<LoopshapingDefinition> loopshapingDefinition) {
  std::unique_ptr<OptimalControlProblem> augmentedProblem(new OptimalControlProblem());

  // Dynamics
  augmentedProblem->dynamics = LoopshapingDynamics::create(*problem.dynamics, loopshapingDefinition);

  // Constraints
  augmentedProblem->equalityConstraint.add("wrapper", LoopshapingConstraint::create(problem.equalityConstraint, loopshapingDefinition));
  augmentedProblem->stateEqualityConstraint.add("wrapper",
                                                LoopshapingConstraint::create(problem.stateEqualityConstraint, loopshapingDefinition));
  augmentedProblem->inequalityConstraint.add("wrapper", LoopshapingConstraint::create(problem.inequalityConstraint, loopshapingDefinition));
  augmentedProblem->preJumpEqualityConstraint.add("wrapper",
                                                  LoopshapingConstraint::create(problem.preJumpEqualityConstraint, loopshapingDefinition));
  augmentedProblem->finalEqualityConstraint.add("wrapper",
                                                LoopshapingConstraint::create(problem.finalEqualityConstraint, loopshapingDefinition));

  // Soft constraints
  // TODO(mspieler): soft constraint wrapper with gamma = 0
  augmentedProblem->cost.add("wrapper", LoopshapingCost::create(problem.cost, loopshapingDefinition));
  augmentedProblem->preJumpCost.add("wrapper", LoopshapingCost::create(problem.preJumpCost, loopshapingDefinition));
  augmentedProblem->finalCost.add("wrapper", LoopshapingCost::create(problem.finalCost, loopshapingDefinition));

  // Cost
  augmentedProblem->cost.add("wrapper", LoopshapingCost::create(problem.cost, loopshapingDefinition));
  augmentedProblem->stateCost.add("wrapper", LoopshapingCost::create(problem.stateCost, loopshapingDefinition));
  augmentedProblem->preJumpCost.add("wrapper", LoopshapingCost::create(problem.preJumpCost, loopshapingDefinition));
  augmentedProblem->finalCost.add("wrapper", LoopshapingCost::create(problem.finalCost, loopshapingDefinition));

  // Pre-computation
  augmentedProblem->preComputation.reset(new LoopshapingPreComputation(problem.preComputation.get(), loopshapingDefinition));

  return augmentedProblem;
}

}  // namespace LoopshapingOptimalControlProblem
}  // namespace ocs2
