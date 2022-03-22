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

#include "ocs2_ddp/unsupported/MPC_OCS2.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_OCS2::MPC_OCS2(const RolloutBase* rolloutPtr, const SystemDynamicsBase* systemDynamicsPtr, const ConstraintBase* systemConstraintsPtr,
                   const CostFunctionBase* costFunctionPtr, const SystemOperatingTrajectoriesBase* operatingTrajectoriesPtr,
                   const scalar_array_t& partitioningTimes, const SLQ_Settings& slqSettings /*= SLQ_Settings()*/,
                   const GDDP_Settings& gddpSettings /*= GDDP_Settings()*/, const MPC_Settings& mpcSettings /*= MPC_Settings()*/,
                   const CostFunctionBase* heuristicsFunctionPtr /*= nullptr*/)

    : MPC_SLQ(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr, partitioningTimes,
              slqSettings, mpcSettings, heuristicsFunctionPtr),
      gddpPtr_(new gddp_t(gddpSettings)),
      activateOCS2_(false),
      terminateOCS2_(false),
      slqDataCollectorPtr_(new slq_data_collector_t(rolloutPtr, systemDynamicsPtr, systemConstraintsPtr, costFunctionPtr)) {
  workerOCS2_ = std::thread(&MPC_OCS2::runOCS2, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MPC_OCS2::~MPC_OCS2() {
  terminateOCS2_ = true;
  ocs2Synchronization_.notify_all();
  workerOCS2_.join();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_OCS2::reset() {
  MPC_SLQ::reset();

  std::lock_guard<std::mutex> ocs2Lock(dataCollectorMutex_);
  activateOCS2_ = false;
  eventTimesOptimized_.clear();
  modeSequenceOptimized_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_OCS2::rewind() {
  MPC_SLQ::rewind();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MPC_OCS2::runOCS2() {
  while (!terminateOCS2_) {
    std::unique_lock<std::mutex> ocs2Lock(dataCollectorMutex_);
    ocs2Synchronization_.wait(ocs2Lock, [&] { return activateOCS2_ || terminateOCS2_; });

    // exit loop
    if (terminateOCS2_) {
      break;
    }

    if (MPC_SLQ::mpcSettings_.debugPrint_) {
      std::cerr << "### OCS2 started. " << std::endl;
    }

    modeSequenceOptimized_ = slqDataCollectorPtr_->modeSchedule_.modeSequence;

    // TODO: fix me
    //    gddpPtr_->run(slqDataCollectorPtr_.get(), eventTimesOptimized_,
    //    MPC_SLQ::mpcSettings_.maxTimeStep_);

    if (MPC_SLQ::mpcSettings_.debugPrint_) {
      std::cerr << "### OCS2 finished. " << std::endl;
    }

    activateOCS2_ = false;
    ocs2Lock.unlock();
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MPC_OCS2::run(const scalar_t& currentTime, const state_vector_t& currentState) {
  std::unique_lock<std::mutex> slqLock(dataCollectorMutex_, std::defer_lock_t());
  bool ownership = slqLock.try_lock();
  if (ownership && !MPC_SLQ::initRun_) {
    bool rewaindTookPlace = currentTime > 0.1 && MPC_SLQ::slqPtr_->getRewindCounter() != slqDataCollectorPtr_->rewindCounter_;
    auto modeSchedule = MPC_SLQ::slqPtr_->getModeSchedule();
    bool modeSequenceUpdated = modeSequenceOptimized_ != modeSchedule.modeSequence;
    if (!rewaindTookPlace && !modeSequenceUpdated) {
      // adjust the SLQ internal controller using trajectory spreading approach
      if (!modeSchedule.eventTimes.empty()) {
        MPC_SLQ::slqPtr_->adjustController(eventTimesOptimized_, modeSchedule.eventTimes);
      }

      // TODO : set eventTimesOptimized_ in the correct place
      // MPC_SLQ::slqPtr_->setModeScheduleManagers({eventTimesOptimized_, modeSchedule.modeSequence});
      //      this->logicRulesPtr_->eventTimes() = eventTimesOptimized_;
      //      this->logicRulesPtr_->update();
      //      this->getLogicRulesMachinePtr()->logicRulesUpdated();
    }

    // collect SLQ variables
    if (MPC_SLQ::mpcSettings_.debugPrint_) {
      std::cerr << "### SLQ data collector triggered." << std::endl;
    }
    slqDataCollectorPtr_->collect(MPC_SLQ::slqPtr_.get());

    activateOCS2_ = true;
    slqLock.unlock();
    ocs2Synchronization_.notify_one();
  }

  // run the base method
  bool status = MPC_SLQ::run(currentTime, currentState);

  return status;
}

}  // namespace ocs2
