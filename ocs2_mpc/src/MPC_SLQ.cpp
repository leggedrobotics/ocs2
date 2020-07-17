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

#include <ocs2_mpc/MPC_SLQ.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_SLQ::MPC_SLQ(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                 const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
                 const scalar_array_t& partitioningTimes, const SLQ_Settings& slqSettings /* = SLQ_Settings()*/,
                 const MPC_Settings& mpcSettings /* = MPC_Settings()*/, const CostFunctionBase* heuristicsFunctionPtr /*= nullptr*/)

    : MPC_BASE(partitioningTimes, mpcSettings) {
  // SLQ
  slqPtr_.reset(new SLQ(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, slqSettings,
                        heuristicsFunctionPtr));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ_Settings& MPC_SLQ::slqSettings() {
  return slqPtr_->settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SLQ* MPC_SLQ::getSolverPtr() {
  return slqPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const SLQ* MPC_SLQ::getSolverPtr() const {
  return slqPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_SLQ::calculateController(const scalar_t& initTime, const vector_t& initState, const scalar_t& finalTime) {
  //*****************************************************************************************
  // updating real-time iteration settings
  //*****************************************************************************************
  // number of iterations
  if (MPC_BASE::initRun_ /*|| slqPtr_->getController().at(MPC_BASE::finalActivePartitionIndex_).empty()*/) {
    slqPtr_->ddpSettings().maxNumIterations_ = MPC_BASE::mpcSettings_.initMaxNumIterations_;
    slqPtr_->ddpSettings().lineSearch_.maxStepLength_ = MPC_BASE::mpcSettings_.initMaxStepLength_;
    slqPtr_->ddpSettings().lineSearch_.minStepLength_ = MPC_BASE::mpcSettings_.initMinStepLength_;
  } else {
    slqPtr_->ddpSettings().maxNumIterations_ = MPC_BASE::mpcSettings_.runtimeMaxNumIterations_;
    slqPtr_->ddpSettings().lineSearch_.maxStepLength_ = MPC_BASE::mpcSettings_.runtimeMaxStepLength_;
    slqPtr_->ddpSettings().lineSearch_.minStepLength_ = MPC_BASE::mpcSettings_.runtimeMinStepLength_;
  }

  // use parallel Riccati solver at each call of realtime-iteration SLQ
  if (!MPC_BASE::initRun_) {
    slqPtr_->useParallelRiccatiSolverFromInitItr(MPC_BASE::mpcSettings_.useParallelRiccatiSolver_ &&
                                                 MPC_BASE::mpcSettings_.recedingHorizon_);
  } else {
    slqPtr_->useParallelRiccatiSolverFromInitItr(false);
  }

  //*****************************************************************************************
  // calculate controller
  //*****************************************************************************************
  if (MPC_BASE::mpcSettings_.coldStart_ || MPC_BASE::initRun_) {
    if (MPC_BASE::mpcSettings_.debugPrint_) {
      std::cerr << "### Using cold initialization." << std::endl;
    }

    slqPtr_->run(initTime, initState, finalTime, MPC_BASE::partitioningTimes_);

  } else {
    slqPtr_->run(initTime, initState, finalTime, MPC_BASE::partitioningTimes_, std::vector<ControllerBase*>());
  }
}

}  // namespace ocs2
