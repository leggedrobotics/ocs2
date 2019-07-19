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
MPC_OCS2<STATE_DIM, INPUT_DIM>::MPC_OCS2()

	: BASE()
	, workerOCS2(&MPC_OCS2::runOCS2, this)
	, activateOCS2_(false)
	, terminateOCS2_(false)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_OCS2<STATE_DIM, INPUT_DIM>::MPC_OCS2(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const scalar_array_t& partitioningTimes,
		const SLQ_Settings& slqSettings /*= SLQ_Settings()*/,
		const MPC_Settings& mpcSettings /*= MPC_Settings()*/,
		std::shared_ptr<HybridLogicRules> logicRulesPtr /*= nullptr*/,
		const mode_sequence_template_t* modeSequenceTemplatePtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: BASE(systemDynamicsPtr,
		   systemDerivativesPtr,
		   systemConstraintsPtr,
		   costFunctionPtr,
		   operatingTrajectoriesPtr,
		   partitioningTimes,
		   slqSettings,
		   mpcSettings,
		   logicRulesPtr,
		   modeSequenceTemplatePtr,
		   heuristicsFunctionPtr)
	, gslqPtr_(new gslq_t(slqSettings))
	, workerOCS2(&MPC_OCS2::runOCS2, this)
	, activateOCS2_(false)
	, terminateOCS2_(false)
	, slqDataCollectorPtr_(new slq_data_collector_t(
			systemDynamicsPtr,
			systemDerivativesPtr,
			systemConstraintsPtr,
			costFunctionPtr))
{
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
MPC_OCS2<STATE_DIM, INPUT_DIM>::~MPC_OCS2() {

	terminateOCS2_ = true;
	ocs2Synchronization_.notify_all();
	workerOCS2.join();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_OCS2<STATE_DIM, INPUT_DIM>::reset() {

	BASE::reset();

	activateOCS2_  = false;
	eventTimesOptimized_.clear();
	subsystemsSequenceOptimized_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_OCS2<STATE_DIM, INPUT_DIM>::rewind() {

	BASE::rewind();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void MPC_OCS2<STATE_DIM, INPUT_DIM>::runOCS2() {

	while (terminateOCS2_==false) {

		std::unique_lock<std::mutex> ocs2Lock(dataCollectorMutex_);
		ocs2Synchronization_.wait(ocs2Lock, [&]{return activateOCS2_ || terminateOCS2_;});

		// exit loop
		if (terminateOCS2_==true) { break;
		}

		if (BASE::mpcSettings_.debugPrint_) {
			std::cerr << "### OCS2 started. " << std::endl;
		}

		subsystemsSequenceOptimized_ = slqDataCollectorPtr_->subsystemsSequence_;

		gslqPtr_->run(
				slqDataCollectorPtr_->eventTimes_,
				slqDataCollectorPtr_.get(),
				eventTimesOptimized_,
				BASE::mpcSettings_.maxTimeStep_);

		if (BASE::mpcSettings_.debugPrint_) {
			std::cerr << "### OCS2 finished. " << std::endl;
		}

		activateOCS2_ = false;
		ocs2Lock.unlock();
	}  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool MPC_OCS2<STATE_DIM, INPUT_DIM>::run(
		const scalar_t& currentTime,
		const state_vector_t& currentState)  {

	std::unique_lock<std::mutex> slqLock(dataCollectorMutex_, std::defer_lock_t());
	bool ownership = slqLock.try_lock();
	if (ownership && BASE::initRun_==false) {

		bool rewaindTookPlace = currentTime>0.1 && BASE::slqPtr_->getRewindCounter() != slqDataCollectorPtr_->rewindCounter_;
		bool modeSequenceUpdated = subsystemsSequenceOptimized_ != BASE::slqPtr_->getLogicRulesPtr()->subsystemsSequence();
		if (!rewaindTookPlace && !modeSequenceUpdated) {

			// adjust the SLQ internal controller using trajectory spreading approach
			if (BASE::slqPtr_->getLogicRulesPtr()->eventTimes().empty()==false) {
				BASE::slqPtr_->adjustController(
						eventTimesOptimized_, BASE::slqPtr_->getLogicRulesPtr()->eventTimes());
			}

			BASE::slqPtr_->getLogicRulesPtr()->eventTimes() = eventTimesOptimized_;
			BASE::slqPtr_->getLogicRulesPtr()->update();
			BASE::slqPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();
		}

		// collect SLQ variables
		if (BASE::mpcSettings_.debugPrint_) {
			std::cerr << "### SLQ data collector triggered." << std::endl;
		}
		slqDataCollectorPtr_->collect(BASE::slqPtr_.get());

		activateOCS2_ = true;
		slqLock.unlock();
		ocs2Synchronization_.notify_one();
	}

	// run the base method
	bool status = BASE::run(currentTime, currentState);

	return status;
}

} // namespace ocs2
