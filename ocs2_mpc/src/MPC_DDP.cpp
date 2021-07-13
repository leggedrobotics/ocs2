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

#include <ocs2_mpc/MPC_DDP.h>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_DDP::MPC_DDP(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                 const CostFunctionBase* costFunctionPtr, const Initializer* initializerPtr, ddp::Settings ddpSettings,
                 mpc::Settings mpcSettings, const CostFunctionBase* heuristicsFunctionPtr /*= nullptr*/)

    : MPC_BASE(std::move(mpcSettings)) {
  switch (ddpSettings.algorithm_) {
    case ddp::Algorithm::SLQ:
      ddpPtr_.reset(new SLQ(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, initializerPtr, std::move(ddpSettings),
                            heuristicsFunctionPtr));
      break;
    case ddp::Algorithm::ILQR:
      ddpPtr_.reset(new ILQR(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, initializerPtr, std::move(ddpSettings),
                             heuristicsFunctionPtr));
      break;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_DDP::calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  // updating real-time iteration settings
  if (MPC_BASE::initRun_ && ddpPtr_->settings().strategy_ == search_strategy::Type::LINE_SEARCH) {
    ddpPtr_->settings().maxNumIterations_ = this->settings().initMaxNumIterations_;
    ddpPtr_->settings().lineSearch_.maxStepLength_ = this->settings().initMaxStepLength_;
    ddpPtr_->settings().lineSearch_.minStepLength_ = this->settings().initMinStepLength_;
  } else {
    ddpPtr_->settings().maxNumIterations_ = this->settings().runtimeMaxNumIterations_;
    ddpPtr_->settings().lineSearch_.maxStepLength_ = this->settings().runtimeMaxStepLength_;
    ddpPtr_->settings().lineSearch_.minStepLength_ = this->settings().runtimeMinStepLength_;
  }

  // calculate controller
  if (this->settings().coldStart_) {
    ddpPtr_->reset();
    ddpPtr_->run(initTime, initState, finalTime, MPC_BASE::partitionTimes_);

  } else {
    if (MPC_BASE::initRun_) {
      ddpPtr_->run(initTime, initState, finalTime, MPC_BASE::partitionTimes_);
    } else {
      ddpPtr_->run(initTime, initState, finalTime, MPC_BASE::partitionTimes_, std::vector<ControllerBase*>());
    }
  }
}

}  // namespace ocs2
