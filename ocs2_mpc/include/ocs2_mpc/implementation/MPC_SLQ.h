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

namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_SLQ()
	: BASE(),
	  initnumPartitions_(0),
	  initPartitioningTimes_(0),
	  numPartitions_(0),
	  partitioningTimes_(0),
	  nullControllersStock_(0),
	  initActivePartitionIndex_(0),
	  finalActivePartitionIndex_(0),
	  lastControlDesignTime_(0.0),
	  optimizedTimeTrajectoriesStock_(0),
	  optimizedStateTrajectoriesStock_(0),
	  optimizedInputTrajectoriesStock_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_SLQ(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const scalar_array_t& partitioningTimes,
		const SLQ_Settings& slqSettings /* = SLQ_Settings()*/,
		const MPC_Settings& mpcSettings /* = MPC_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /* = nullptr*/,
		const mode_sequence_template_t* modeSequenceTemplatePtr /* = nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: BASE(mpcSettings)
	, initnumPartitions_(partitioningTimes.size()-1)
	, initPartitioningTimes_(partitioningTimes)
	, numPartitions_(0)
	, partitioningTimes_(0)
	, nullControllersStock_(0)
	, initActivePartitionIndex_(0)
	, finalActivePartitionIndex_(0)
	, lastControlDesignTime_(partitioningTimes.front())
	, optimizedTimeTrajectoriesStock_(0)
	, optimizedStateTrajectoriesStock_(0)
	, optimizedInputTrajectoriesStock_(0)

{
	if (partitioningTimes.size() < 2)
		throw std::runtime_error("There should be at least one time partition.");

	if (mpcSettings.recedingHorizon_==true) {
		numPartitions_ = 3*initnumPartitions_;
		partitioningTimes_.clear();

		const scalar_t timeHorizon = initPartitioningTimes_.back() - initPartitioningTimes_.front();

		for (size_t j=0; j<3; j++) {
			for (size_t i=0; i<initnumPartitions_; i++) {
				partitioningTimes_.push_back( ((int)j-1)*timeHorizon + initPartitioningTimes_[i] );
			} // end of i loop
		} // end of j loop
		partitioningTimes_.push_back(initPartitioningTimes_[initnumPartitions_] + timeHorizon);

	} else {
		numPartitions_  = initnumPartitions_;
		partitioningTimes_ = initPartitioningTimes_;
	}

	nullControllersStock_.resize(numPartitions_);

	// SLQP
	if (slqSettings.useMultiThreading_==true) {
		slqPtr_.reset( new slq_mp_t(
				systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
				slqSettings, logicRulesPtr, heuristicsFunctionPtr) );
	} else {
		slqPtr_.reset( new slq_t(
				systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
				slqSettings, logicRulesPtr, heuristicsFunctionPtr) );
	}

	// SLQ-MPC activates this if the final time of the MPC will increase by the length of a time partition instead
	// of commonly used scheme where the final time is gradual increased.
	slqPtr_->blockwiseMovingHorizon(mpcSettings.blockwiseMovingHorizon_);

	// set mode sequence template
	if (modeSequenceTemplatePtr)
		slqPtr_->getLogicRules().setModeSequenceTemplate(*modeSequenceTemplatePtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

	BASE::reset();
	slqPtr_->reset();
	// reset it if time is reset in MRT
//	lastControlDesignTime_ = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStartTime()  const {

	return lastControlDesignTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalTime() const {

	if (BASE::mpcSettings_.recedingHorizon_ == true)
		return lastControlDesignTime_ + getTimeHorizon();
	else
		return initPartitioningTimes_.back();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getTimeHorizon() const {

	if (BASE::mpcSettings_.recedingHorizon_ == true)
		return initPartitioningTimes_.back() - initPartitioningTimes_.front();
	else
		return initPartitioningTimes_.back() - lastControlDesignTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_Settings& MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::slqSettings() {

	return slqPtr_->settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPartitioningTimes(
		scalar_array_t& partitioningTimes) const {

	partitioningTimes.resize(finalActivePartitionIndex_+2);
	for (size_t i=0; i<=finalActivePartitionIndex_+1; i++ )
		partitioningTimes[i] = partitioningTimes_[i];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getEventTimes() const {

	return slqPtr_->getEventTimes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(
		const LOGIC_RULES_T& logicRules) {

	slqPtr_->setLogicRules(logicRules);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T& MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRules() const {

	return slqPtr_->getLogicRules();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rewind() {

	for (size_t i=0; i<2*initnumPartitions_; i++) {
		partitioningTimes_[i] = partitioningTimes_[i+initnumPartitions_];
	}

	const scalar_t timeHorizon = initPartitioningTimes_.back() - initPartitioningTimes_.front();
	for (size_t i=0; i<initnumPartitions_; i++) {
		partitioningTimes_[i+2*initnumPartitions_] = 2.0*timeHorizon + partitioningTimes_[i];
	}
	partitioningTimes_[3*initnumPartitions_] = 3.0*timeHorizon + partitioningTimes_[0];

	// SLQ internal variables
	slqPtr_->rewindOptimizer(initnumPartitions_);

	// Rewind the internal SLQ's logicRules. It is not necessary to call the
	// LogicRulesMachine::updateLogicRules() method, since the partitioningTimes_
	// is updated as well the update method will be called automatically.
	// And controller are adjusted separately as well.
	slqPtr_->getLogicRules().rewind(partitioningTimes_.front(), partitioningTimes_.back());
//	slqPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::adjustmentTimeHorizon(
		const scalar_array_t& partitioningTimes,
		scalar_t& initTime,
		scalar_t& finalTime,
		size_t& initActivePartitionIndex,
		size_t& finalActivePartitionIndex) const {

	// current active subsystem
	initActivePartitionIndex = slq_base_t::findActivePartitionIndex(partitioningTimes, initTime);

	if (initTime > partitioningTimes[initActivePartitionIndex+1] - 4e-3) {
		initTime = partitioningTimes[initActivePartitionIndex+1] + 1e-5;
		initActivePartitionIndex++;
	}

	// final active subsystem
	finalActivePartitionIndex = slq_base_t::findActivePartitionIndex(partitioningTimes, finalTime);

	// if it is at the very beginning of the partition (4e-3) reduce the final time to
	// last partition final time otherwise set to the final time of the current partition
	// (if blockwiseMovingHorizon is active)
	if (finalTime < partitioningTimes[finalActivePartitionIndex] + 4e-3) {
		finalTime = partitioningTimes[finalActivePartitionIndex];
		finalActivePartitionIndex--;
	} else {
		if (BASE::mpcSettings_.blockwiseMovingHorizon_==true)
			finalTime = partitioningTimes[finalActivePartitionIndex+1];
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostDesiredTrajectoriesPtr(
		const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const {

	slqPtr_->getCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const cost_desired_trajectories_t& costDesiredTrajectories) {

	slqPtr_->setCostDesiredTrajectories(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const scalar_array_t& desiredTimeTrajectory,
		const dynamic_vector_array_t& desiredStateTrajectory,
		const dynamic_vector_array_t& desiredInputTrajectory) {

	slqPtr_->setCostDesiredTrajectories(desiredTimeTrajectory,
			desiredStateTrajectory, desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		cost_desired_trajectories_t& costDesiredTrajectories) {

	slqPtr_->swapCostDesiredTrajectories(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		scalar_array_t& desiredTimeTrajectory,
		dynamic_vector_array_t& desiredStateTrajectory,
		dynamic_vector_array_t& desiredInputTrajectory) {

	slqPtr_->swapCostDesiredTrajectories(desiredTimeTrajectory,
			desiredStateTrajectory, desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController(
		scalar_t& initTime,
		const state_vector_t& initState,
		scalar_t& finalTime,
		const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr,
		const controller_array_t*& controllerStockPtr)  {


	//******************************************************************************************
	// rewind the optimizer
	//******************************************************************************************
	if (finalTime>partitioningTimes_.back() /*&& BASE::mpcSettings_.recedingHorizon_==true*/) {
		if (BASE::mpcSettings_.debugPrint_)
			std::cerr << "### SLQ is rewinded at time " << initTime << " [s]." << std::endl;

		this->rewind();
	}

	//*****************************************************************************************
	// time horizon adjustment
	//*****************************************************************************************
	this->adjustmentTimeHorizon(partitioningTimes_,
			initTime, finalTime,
			initActivePartitionIndex_, finalActivePartitionIndex_);

	lastControlDesignTime_ = initTime;

	// display
	if (BASE::mpcSettings_.debugPrint_) {
		std::cerr << "### MPC number of subsystems: " << finalActivePartitionIndex_-initActivePartitionIndex_+1 << "." << std::endl;
		std::cerr << "### MPC time horizon is adjusted." << std::endl;
		std::cerr << "\t### MPC final Time:   " << finalTime << " [s]." << std::endl;
		std::cerr << "\t### MPC time horizon: " << finalTime-initTime << " [s]." << std::endl;
	}

	//*****************************************************************************************
	// cost goal check
	//*****************************************************************************************
	if (BASE::initRun_==true && slqPtr_->costDesiredTrajectoriesUpdated()==false) {
		std::cerr << "### WARNING: The initial desired trajectories are not set. "
				"This may cause undefined behavior. Use the MPC_SLQ::setCostDesiredTrajectories() "
				"method to provide appropriate goal trajectories." << std::endl;
	}

	//*****************************************************************************************
	// Updating some settings
	//*****************************************************************************************
	// number of iterations
	if (BASE::initRun_==true /*|| slqPtr_->getController().at(finalActivePartitionIndex_).empty()==true*/) {
		slqPtr_->settings().maxNumIterationsSLQ_  = BASE::mpcSettings_.initMaxNumIterations_;
		slqPtr_->settings().maxLearningRateGSLQP_ = BASE::mpcSettings_.initMaxLearningRate_;
		slqPtr_->settings().minLearningRateGSLQP_ = BASE::mpcSettings_.initMinLearningRate_;
	} else {
		slqPtr_->settings().maxNumIterationsSLQ_  = BASE::mpcSettings_.runtimeNumIterations_;
		slqPtr_->settings().maxLearningRateGSLQP_ = BASE::mpcSettings_.runtimeLearningRate_;
		slqPtr_->settings().minLearningRateGSLQP_ = BASE::mpcSettings_.runtimeLearningRate_;
	}

	// use parallel Riccati solver at each call of realtime-iteration SLQ
	if (BASE::initRun_==false) {
		if (BASE::mpcSettings_.useParallelRiccatiSolver_==true && BASE::mpcSettings_.recedingHorizon_==true)
			slqPtr_->useParallelRiccatiSolverFromInitItr(true);
		else
			slqPtr_->useParallelRiccatiSolverFromInitItr(false);
	} else {
		slqPtr_->useParallelRiccatiSolverFromInitItr(false);
	}

	//*****************************************************************************************
	// calculate controller
	//*****************************************************************************************
	if (BASE::mpcSettings_.coldStart_==true || BASE::initRun_==true) {

		if (BASE::mpcSettings_.debugPrint_)
			std::cerr << "### Using cold initialization." << std::endl;

		slqPtr_->run(initTime, initState, finalTime, partitioningTimes_);

	} else {
		slqPtr_->run(initTime, initState, finalTime, partitioningTimes_,
				typename slq_base_t::INTERNAL_CONTROLLER());
	}

	//*****************************************************************************************
	// Get optimized outputs
	//*****************************************************************************************
	// swap the optimized trajectories
	optimizedTimeTrajectoriesStock_.clear();
	optimizedStateTrajectoriesStock_.clear();
	optimizedInputTrajectoriesStock_.clear();
	slqPtr_->swapNominalTrajectories(
			optimizedTimeTrajectoriesStock_,
			optimizedStateTrajectoriesStock_,
			optimizedInputTrajectoriesStock_);
	timeTrajectoriesStockPtr  = &optimizedTimeTrajectoriesStock_;
	stateTrajectoriesStockPtr = &optimizedStateTrajectoriesStock_;
	inputTrajectoriesStockPtr = &optimizedInputTrajectoriesStock_;

	// get the optimal controller
	slqPtr_->getControllerPtr(controllerStockPtr);

	//*****************************************************************************************
	// Update logicRules if new logicRulesTemplate is set
	//*****************************************************************************************
	if (BASE::logicRulesTemplateUpdated_ == true) {

		// set new templates
		slqPtr_->getLogicRules().setModeSequenceTemplate(BASE::newLogicRulesTemplate_);
		slqPtr_->getLogicRules().insertModeSequenceTemplate(finalTime, partitioningTimes_.back());
		slqPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();

		BASE::logicRulesTemplateUpdated_ = false;
	}

}

} // namespace ocs2

