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

#include <ocs2_core/loopshaping/LoopshapingDefinition.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2 {

/**
 * This wraps SolverSynchronizedModules and applies the loopshaping conversion before forwarding calls.
 */
class LoopshapingSynchronizedModule : public SolverSynchronizedModule {
 public:
  LoopshapingSynchronizedModule(std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr,
                                std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulesPtrArray);

  ~LoopshapingSynchronizedModule() override = default;

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override;

  void add(std::shared_ptr<ocs2::SolverSynchronizedModule> module) { synchronizedModulesPtrArray_.push_back(std::move(module)); }

 private:
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr_;
  std::vector<std::shared_ptr<SolverSynchronizedModule>> synchronizedModulesPtrArray_;
};

}  // namespace ocs2
