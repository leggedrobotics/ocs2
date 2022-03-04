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

#include <iostream>
#include <mutex>

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/Numerics.h>

#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SolverBase::SolverBase() : referenceManagerPtr_(new ReferenceManager) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  preRun(initTime, initState, finalTime);
  runImpl(initTime, initState, finalTime);
  postRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::run(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) {
  preRun(initTime, initState, finalTime);
  runImpl(initTime, initState, finalTime, externalControllerPtr);
  postRun();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PrimalSolution SolverBase::primalSolution(scalar_t finalTime) const {
  PrimalSolution primalSolution;
  getPrimalSolution(finalTime, &primalSolution);
  return primalSolution;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::printString(const std::string& text) const {
  std::lock_guard<std::mutex> outputDisplayGuard(outputDisplayGuardMutex_);
  std::cerr << text << '\n';
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::preRun(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  referenceManagerPtr_->preSolverRun(initTime, finalTime, initState);

  for (auto& module : synchronizedModules_) {
    module->preSolverRun(initTime, finalTime, initState, *referenceManagerPtr_);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SolverBase::postRun() {
  if (!synchronizedModules_.empty()) {
    const auto solution = primalSolution(getFinalTime());
    for (auto& module : synchronizedModules_) {
      module->postSolverRun(solution);
    }
  }
}

}  // namespace ocs2
