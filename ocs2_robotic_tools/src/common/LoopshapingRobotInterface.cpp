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

#include "ocs2_robotic_tools/common/LoopshapingRobotInterface.h"

#include <ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LoopshapingRobotInterface::LoopshapingRobotInterface(std::unique_ptr<RobotInterface> robotInterfacePtr,
                                                     std::shared_ptr<LoopshapingDefinition> loopshapingDefinitionPtr)
    : robotInterfacePtr_(std::move(robotInterfacePtr)), loopshapingDefinitionPtr_(std::move(loopshapingDefinitionPtr)) {
  // wrap with loopshaping
  optimalControlProblem_ =
      LoopshapingOptimalControlProblem::create(robotInterfacePtr_->getOptimalControlProblem(), loopshapingDefinitionPtr_);

  initializerPtr_.reset(new ocs2::LoopshapingInitializer(robotInterfacePtr_->getInitializer(), loopshapingDefinitionPtr_));

  if (robotInterfacePtr_->getReferenceManagerPtr() != nullptr) {
    loopshapingReferenceManager_ =
        std::make_shared<LoopshapingReferenceManager>(robotInterfacePtr_->getReferenceManagerPtr(), loopshapingDefinitionPtr_);
  }
}

}  // namespace ocs2
