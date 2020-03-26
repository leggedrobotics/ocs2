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

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_SLQ<STATE_DIM, INPUT_DIM>::MPC_SLQ(const rollout_base_t* rolloutPtr, const derivatives_base_t* systemDerivativesPtr,
                                       const constraint_base_t* systemConstraintsPtr, const cost_function_base_t* costFunctionPtr,
                                       const operating_trajectories_base_t* operatingTrajectoriesPtr,
                                       const scalar_array_t& partitioningTimes, const SLQ_Settings& slqSettings /* = SLQ_Settings()*/,
                                       const MPC_Settings& mpcSettings /* = MPC_Settings()*/,
                                       const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

    : BASE(partitioningTimes, mpcSettings) {
  // SLQ
  slqPtr_.reset(new slq_t(rolloutPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, slqSettings,
                          heuristicsFunctionPtr));

  // set base solver's pointer
  BASE::setBaseSolverPtr(slqPtr_.get());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQ_Settings& MPC_SLQ<STATE_DIM, INPUT_DIM>::slqSettings() {
  return slqPtr_->settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename MPC_SLQ<STATE_DIM, INPUT_DIM>::slq_t* MPC_SLQ<STATE_DIM, INPUT_DIM>::getSolverPtr() {
  return slqPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename MPC_SLQ<STATE_DIM, INPUT_DIM>::slq_t* MPC_SLQ<STATE_DIM, INPUT_DIM>::getSolverPtr() const {
  return slqPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_SLQ<STATE_DIM, INPUT_DIM>::calculateController(const scalar_t& initTime, const state_vector_t& initState,
                                                        const scalar_t& finalTime) {
  //*****************************************************************************************
  // updating real-time iteration settings
  //*****************************************************************************************
  // number of iterations
  if (BASE::initRun_ /*|| slqPtr_->getController().at(BASE::finalActivePartitionIndex_).empty()*/) {
    slqPtr_->ddpSettings().maxNumIterations_ = BASE::mpcSettings_.initMaxNumIterations_;
    slqPtr_->ddpSettings().maxLearningRate_ = BASE::mpcSettings_.initMaxLearningRate_;
    slqPtr_->ddpSettings().minLearningRate_ = BASE::mpcSettings_.initMinLearningRate_;
  } else {
    slqPtr_->ddpSettings().maxNumIterations_ = BASE::mpcSettings_.runtimeMaxNumIterations_;
    slqPtr_->ddpSettings().maxLearningRate_ = BASE::mpcSettings_.runtimeMaxLearningRate_;
    slqPtr_->ddpSettings().minLearningRate_ = BASE::mpcSettings_.runtimeMinLearningRate_;
  }

  // use parallel Riccati solver at each call of realtime-iteration SLQ
  if (!BASE::initRun_) {
    slqPtr_->useParallelRiccatiSolverFromInitItr(BASE::mpcSettings_.useParallelRiccatiSolver_ && BASE::mpcSettings_.recedingHorizon_);
  } else {
    slqPtr_->useParallelRiccatiSolverFromInitItr(false);
  }

  //*****************************************************************************************
  // calculate controller
  //*****************************************************************************************
  if (BASE::mpcSettings_.coldStart_ || BASE::initRun_) {
    if (BASE::mpcSettings_.debugPrint_) {
      std::cerr << "### Using cold initialization." << std::endl;
    }

    slqPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_);

  } else {
    slqPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_, typename slq_t::controller_ptr_array_t());
  }
}

}  // namespace ocs2
