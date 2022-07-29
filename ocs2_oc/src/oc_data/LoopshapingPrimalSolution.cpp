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

#include "ocs2_oc/oc_data/LoopshapingPrimalSolution.h"

namespace ocs2 {

PrimalSolution loopshapingToSystemPrimalSolution(const PrimalSolution& primalSolution, const LoopshapingDefinition& loopshapingDefinition) {
  PrimalSolution systemPrimalSolution;
  systemPrimalSolution.controllerPtr_ = nullptr;
  systemPrimalSolution.timeTrajectory_ = primalSolution.timeTrajectory_;
  systemPrimalSolution.modeSchedule_ = primalSolution.modeSchedule_;
  systemPrimalSolution.stateTrajectory_.reserve(primalSolution.stateTrajectory_.size());
  systemPrimalSolution.inputTrajectory_.reserve(primalSolution.inputTrajectory_.size());
  for (size_t k = 0; k < primalSolution.stateTrajectory_.size(); ++k) {
    systemPrimalSolution.stateTrajectory_.push_back(loopshapingDefinition.getSystemState(primalSolution.stateTrajectory_[k]));
    systemPrimalSolution.inputTrajectory_.push_back(
        loopshapingDefinition.getSystemInput(primalSolution.stateTrajectory_[k], primalSolution.inputTrajectory_[k]));
  }
  return systemPrimalSolution;
}

}  // namespace ocs2
