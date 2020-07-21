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

#include <ocs2_mpc/MPC_ILQR.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_ILQR::MPC_ILQR(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                   const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
                   scalar_t timeHorizon, size_t numPartitions, const ILQR_Settings& ilqrSettings /* = ILQR_Settings()*/,
                   const MPC_Settings& mpcSettings /* = MPC_Settings()*/, const CostFunctionBase* heuristicsFunctionPtr /*= nullptr*/)

    : MPC_BASE(timeHorizon, numPartitions, mpcSettings) {
  ilqrPtr_.reset(new ILQR(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, ilqrSettings,
                          heuristicsFunctionPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_ILQR::calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  // updating real-time iteration settings
  if (MPC_BASE::initRun_) {
    ilqrPtr_->ddpSettings().maxNumIterations_ = this->settings().initMaxNumIterations_;
    ilqrPtr_->ddpSettings().lineSearch_.maxStepLength_ = this->settings().initMaxStepLength_;
    ilqrPtr_->ddpSettings().lineSearch_.minStepLength_ = this->settings().initMinStepLength_;
  } else {
    ilqrPtr_->ddpSettings().maxNumIterations_ = this->settings().runtimeMaxNumIterations_;
    ilqrPtr_->ddpSettings().lineSearch_.maxStepLength_ = this->settings().runtimeMaxStepLength_;
    ilqrPtr_->ddpSettings().lineSearch_.minStepLength_ = this->settings().runtimeMinStepLength_;
  }

  // calculate controller
  if (this->settings().coldStart_ || MPC_BASE::initRun_) {
    if (this->settings().debugPrint_) {
      std::cerr << "### Using cold initialization.\n";
    }
    ilqrPtr_->run(initTime, initState, finalTime, MPC_BASE::partitionTimes_);
  } else {
    ilqrPtr_->run(initTime, initState, finalTime, MPC_BASE::partitionTimes_, std::vector<ControllerBase*>());
  }
}

}  // namespace ocs2
