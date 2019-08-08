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

#include <ocs2_oc/oc_solver/Solver_BASE.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
Solver_BASE<STATE_DIM, INPUT_DIM>::Solver_BASE(std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr */)
    : costDesiredTrajectoriesUpdated_(false) {
  if (!logicRulesPtr) {
    logicRulesPtr = std::shared_ptr<HybridLogicRules>(new NullLogicRules());
  }
  logicRulesMachinePtr_ = logic_rules_machine_ptr_t(new logic_rules_machine_t(std::move(logicRulesPtr)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::setCostDesiredTrajectories(const cost_desired_trajectories_t& costDesiredTrajectories) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesBuffer_ = costDesiredTrajectories;
  costDesiredTrajectoriesUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::setCostDesiredTrajectories(const scalar_array_t& desiredTimeTrajectory,
                                                                   const dynamic_vector_array_t& desiredStateTrajectory,
                                                                   const dynamic_vector_array_t& desiredInputTrajectory) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesBuffer_.desiredTimeTrajectory() = desiredTimeTrajectory;
  costDesiredTrajectoriesBuffer_.desiredStateTrajectory() = desiredStateTrajectory;
  costDesiredTrajectoriesBuffer_.desiredInputTrajectory() = desiredInputTrajectory;
  costDesiredTrajectoriesUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::swapCostDesiredTrajectories(cost_desired_trajectories_t& costDesiredTrajectories) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesBuffer_.swap(costDesiredTrajectories);
  costDesiredTrajectoriesUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::swapCostDesiredTrajectories(scalar_array_t& desiredTimeTrajectory,
                                                                    dynamic_vector_array_t& desiredStateTrajectory,
                                                                    dynamic_vector_array_t& desiredInputTrajectory) {
  std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
  costDesiredTrajectoriesBuffer_.desiredTimeTrajectory().swap(desiredTimeTrajectory);
  costDesiredTrajectoriesBuffer_.desiredStateTrajectory().swap(desiredStateTrajectory);
  costDesiredTrajectoriesBuffer_.desiredInputTrajectory().swap(desiredInputTrajectory);
  costDesiredTrajectoriesUpdated_ = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool Solver_BASE<STATE_DIM, INPUT_DIM>::updateCostDesiredTrajectories() {
  if (costDesiredTrajectoriesUpdated_) {
    std::lock_guard<std::mutex> lock(costDesiredTrajectoriesBufferMutex_);
    costDesiredTrajectoriesUpdated_ = false;
    costDesiredTrajectories_.swap(costDesiredTrajectoriesBuffer_);
    return true;
  }
  return false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::printString(const std::string& text) {
  std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
  std::cerr << text << std::endl;
}

}  // namespace ocs2
