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
void Solver_BASE<STATE_DIM, INPUT_DIM>::run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                                            const scalar_array_t& partitioningTimes) {
  preRun(initTime, initState, finalTime);
  runImpl(initTime, initState, finalTime, partitioningTimes);
  postRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::run(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime,
                                            const scalar_array_t& partitioningTimes, const controller_ptr_array_t& controllersPtrStock) {
  preRun(initTime, initState, finalTime);
  runImpl(initTime, initState, finalTime, partitioningTimes, controllersPtrStock);
  postRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
auto Solver_BASE<STATE_DIM, INPUT_DIM>::primalSolution(scalar_t finalTime) const -> primal_solution_t {
  primal_solution_t primalSolution;
  getPrimalSolution(finalTime, &primalSolution);
  return primalSolution;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::printString(const std::string& text) {
  std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
  std::cerr << text << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::preRun(scalar_t initTime, const state_vector_t& initState, scalar_t finalTime) {
  if (modeScheduleManager_) {
    modeScheduleManager_->preSolverRun(initTime, finalTime, initState, costDesiredTrajectories_);
    modeSchedule_ = modeScheduleManager_->getModeSchedule();
  }
  for (auto& module : synchronizedModules_) {
    module->preSolverRun(initTime, finalTime, initState, costDesiredTrajectories_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void Solver_BASE<STATE_DIM, INPUT_DIM>::postRun() {
  if (modeScheduleManager_ || !synchronizedModules_.empty()) {
    const auto solution = primalSolution(getFinalTime());
    if (modeScheduleManager_) {
      modeScheduleManager_->postSolverRun(solution);
    }
    for (auto& module : synchronizedModules_) {
      module->postSolverRun(solution);
    }
  }
}

}  // namespace ocs2
