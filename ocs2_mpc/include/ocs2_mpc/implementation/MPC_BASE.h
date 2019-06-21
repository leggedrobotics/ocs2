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

using namespace std::chrono;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_BASE()

	: initRun_(true)
	, logicRulesTemplateUpdated_(false)
	, optimizedControllersStockPtr_(nullptr)
	, optimizedTimeTrajectoriesStockPtr_(nullptr)
	, optimizedStateTrajectoriesStockPtr_(nullptr)
	, optimizedInputTrajectoriesStockPtr_(nullptr)
	, measuredRuntimeMS_(0)
	, initnumPartitions_(0)
	, initPartitioningTimes_(0)
	, numPartitions_(0)
	, partitioningTimes_(0)
	, initActivePartitionIndex_(0)
	, finalActivePartitionIndex_(0)
	, lastControlDesignTime_(0.0)
	, solverPtr_(nullptr)

{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_BASE(
		const scalar_array_t& partitioningTimes,
		const MPC_Settings &mpcSettings /*= MPC_Settings()*/)

	: mpcSettings_(mpcSettings)
	, initRun_(true)
	, logicRulesTemplateUpdated_(false)
	, optimizedControllersStockPtr_(nullptr)
	, optimizedTimeTrajectoriesStockPtr_(nullptr)
	, optimizedStateTrajectoriesStockPtr_(nullptr)
	, optimizedInputTrajectoriesStockPtr_(nullptr)
	, measuredRuntimeMS_(0)
	, initnumPartitions_(partitioningTimes.size()-1)
	, initPartitioningTimes_(partitioningTimes)
	, numPartitions_(0)
	, partitioningTimes_(0)
	, initActivePartitionIndex_(0)
	, finalActivePartitionIndex_(0)
	, lastControlDesignTime_(partitioningTimes.front())
	, solverPtr_(nullptr)

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
		numPartitions_ = initnumPartitions_;
		partitioningTimes_ = initPartitioningTimes_;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

	initRun_ = true;
	measuredRuntimeMS_ = std::chrono::milliseconds(0);
	logicRulesTemplateUpdated_ = false;
	optimizedControllersStockPtr_ = nullptr;
	optimizedTimeTrajectoriesStockPtr_ = nullptr;
	optimizedStateTrajectoriesStockPtr_ = nullptr;
	optimizedInputTrajectoriesStockPtr_ = nullptr;

	solverPtr_->reset();
	// reset it if time is reset in MRT
//	lastControlDesignTime_ = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setBaseSolverPtr(solver_base_t* solverPtr) {

	solverPtr_ = solverPtr;

	// MPC activates this if the final time of the MPC will increase by the length of a time partition instead
	// of commonly used scheme where the final time is gradual increased.
	solverPtr_->blockwiseMovingHorizon(mpcSettings_.blockwiseMovingHorizon_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rewind() {

	for (size_t i=0; i<2*initnumPartitions_; i++) {
		partitioningTimes_[i] = partitioningTimes_[i+initnumPartitions_];
	}

	const scalar_t timeHorizon = initPartitioningTimes_.back() - initPartitioningTimes_.front();
	for (size_t i=0; i<initnumPartitions_; i++) {
		partitioningTimes_[i+2*initnumPartitions_] = 2.0*timeHorizon + partitioningTimes_[i];
	}
	partitioningTimes_[3*initnumPartitions_] = 3.0*timeHorizon + partitioningTimes_[0];

	// Solver internal variables
	solverPtr_->rewindOptimizer(initnumPartitions_);

	// Rewind the solver's internal logicRules. It is not necessary to call the
	// LogicRulesMachine::updateLogicRules() method, since the partitioningTimes_
	// is updated as well the update method will be called automatically.
	if (solverPtr_->getLogicRulesPtr())
		solverPtr_->getLogicRulesPtr()->rewind(partitioningTimes_.front(), partitioningTimes_.back());
//	if (solverPtr_->getLogicRulesMachinePtr())
//		solverPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::adjustmentTimeHorizon(
		const scalar_array_t& partitioningTimes,
		scalar_t& initTime,
		scalar_t& finalTime,
		size_t& initActivePartitionIndex,
		size_t& finalActivePartitionIndex) const {

	// current active subsystem
	initActivePartitionIndex = solver_base_t::findActivePartitionIndex(partitioningTimes, initTime);

	if (initTime > partitioningTimes[initActivePartitionIndex+1] - 4e-3) {
		initTime = partitioningTimes[initActivePartitionIndex+1] + 1e-5;
		initActivePartitionIndex++;
	}

	// final active subsystem
	finalActivePartitionIndex = solver_base_t::findActivePartitionIndex(partitioningTimes, finalTime);

	// if it is at the very beginning of the partition (4e-3) reduce the final time to
	// last partition final time otherwise set to the final time of the current partition
	// (if blockwiseMovingHorizon is active)
	if (finalTime < partitioningTimes[finalActivePartitionIndex] + 4e-3) {
		finalTime = partitioningTimes[finalActivePartitionIndex];
		finalActivePartitionIndex--;
	} else {
		if (mpcSettings_.blockwiseMovingHorizon_==true)
			finalTime = partitioningTimes[finalActivePartitionIndex+1];
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& currentTime,
		const state_vector_t& currentState)  {

	// check if the current time exceeds the solver final limit
	if (currentTime>=getFinalTime() && mpcSettings_.recedingHorizon_==true) {

		if (initRun_==true){
		  for (int i=0; i<partitioningTimes_.size(); i++){
		    partitioningTimes_[i] += currentTime;
		  }
		}
		else {
      std::cerr << std::endl << "#####################################################";
      std::cerr << std::endl << "#####################################################";
      std::cerr << std::endl << "#####################################################" << std::endl;
      std::cerr << "### MPC is called at time:  " << currentTime << " [s]." << std::endl;
      std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time." << std::endl;
      std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << getFinalTime() << std::endl;

      return false;
    }
	}

	// display
	if (mpcSettings_.debugPrint_) {
		std::cerr << std::endl << "#####################################################";
		std::cerr << std::endl << "#####################################################";
		std::cerr << std::endl << "#####################################################" << std::endl;
		std::cerr << "### MPC is called at time:  " << currentTime << " [s]." << std::endl;
		// CPU time at the beginning MPC
		mpcStratTime_ = high_resolution_clock::now();
	}


	/******************************************************************************************
	 * Determine MPC time horizon
	 ******************************************************************************************/
	scalar_t initTime = currentTime;
	scalar_t finalTime;
	if (mpcSettings_.recedingHorizon_==true)
		finalTime = currentTime + getTimeHorizon();
	else {
		size_t N = currentTime / getFinalTime();
		finalTime = (N+1) * getFinalTime();
	}

	if (mpcSettings_.debugPrint_) {
		std::cerr << "### MPC final Time:         " << finalTime << " [s]." << std::endl;
		std::cerr << "### MPC time horizon:       " << finalTime-currentTime << " [s]." << std::endl;
	}

	/******************************************************************************************
	* rewind the optimizer
	******************************************************************************************/
	if (finalTime>partitioningTimes_.back() /*&& BASE::mpcSettings_.recedingHorizon_==true*/) {
		if (mpcSettings_.debugPrint_)
			std::cerr << "### MPC is rewinded at time " << currentTime << " [s]." << std::endl;

		rewind();
	}

	/******************************************************************************************
	* time horizon adjustment
	******************************************************************************************/
	adjustmentTimeHorizon(partitioningTimes_,
			initTime, finalTime,
			initActivePartitionIndex_, finalActivePartitionIndex_);

	lastControlDesignTime_ = initTime;

	// display
	if (mpcSettings_.debugPrint_) {
		std::cerr << "### MPC number of subsystems: " << finalActivePartitionIndex_-initActivePartitionIndex_+1 << "." << std::endl;
		std::cerr << "### MPC time horizon is adjusted." << std::endl;
		std::cerr << "\t### MPC final Time:   " << finalTime << " [s]." << std::endl;
		std::cerr << "\t### MPC time horizon: " << finalTime-initTime << " [s]." << std::endl;
	}

	//*****************************************************************************************
	// Update logicRules if new logicRulesTemplate is set
	//*****************************************************************************************
	if (logicRulesTemplateUpdated_ == true) {

		// set new templates
		solverPtr_->getLogicRulesPtr()->setModeSequenceTemplate(newLogicRulesTemplate_);
		solverPtr_->getLogicRulesPtr()->insertInternalModeSequenceTemplate(finalTime, partitioningTimes_.back());
		solverPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();

		logicRulesTemplateUpdated_ = false;
	}

	/******************************************************************************************
	 * Calculate controller
	 ******************************************************************************************/
	// calculate the MPC controller
	calculateController(initTime, currentState, finalTime,
			optimizedTimeTrajectoriesStockPtr_,
			optimizedStateTrajectoriesStockPtr_,
			optimizedInputTrajectoriesStockPtr_,
			optimizedControllersStockPtr_);

	// set initRun flag to false
	initRun_ = false;

	// display
	if (mpcSettings_.debugPrint_) {
		// updating runtime of the MPC for adaptive frequency
		measuredRuntimeMS_ = duration_cast<milliseconds>(high_resolution_clock::now() - mpcStratTime_);
		std::cerr << "### Measured runtime delay: " << measuredRuntimeMS_.count() << "[ms]."<< std::endl;
	}

	return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getStartTime() const {

	return lastControlDesignTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getFinalTime() const {

	if (mpcSettings_.recedingHorizon_ == true)
		return lastControlDesignTime_ + getTimeHorizon();
	else
		return initPartitioningTimes_.back();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
	MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getTimeHorizon() const {

	if (mpcSettings_.recedingHorizon_ == true)
		return initPartitioningTimes_.back() - initPartitioningTimes_.front();
	else
		return initPartitioningTimes_.back() - lastControlDesignTime_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPartitioningTimes(
		scalar_array_t& partitioningTimes) const {

	partitioningTimes.resize(finalActivePartitionIndex_+2);
	for (size_t i=0; i<=finalActivePartitionIndex_+1; i++ )
		partitioningTimes[i] = partitioningTimes_[i];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(
		const LOGIC_RULES_T& logicRules) {

	solverPtr_->setLogicRules(logicRules);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const LOGIC_RULES_T* MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getLogicRulesPtr() const {

	return solverPtr_->getLogicRulesPtr();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setNewLogicRulesTemplate(
		const mode_sequence_template_t& newLogicRulesTemplate) {

	logicRulesTemplateUpdated_ = true;
	newLogicRulesTemplate_ = newLogicRulesTemplate;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostDesiredTrajectoriesPtr(
		const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr) const {

	solverPtr_->getCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const cost_desired_trajectories_t& costDesiredTrajectories) {

	solverPtr_->setCostDesiredTrajectories(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setCostDesiredTrajectories(
		const scalar_array_t& desiredTimeTrajectory,
		const dynamic_vector_array_t& desiredStateTrajectory,
		const dynamic_vector_array_t& desiredInputTrajectory) {

	solverPtr_->setCostDesiredTrajectories(desiredTimeTrajectory,
			desiredStateTrajectory, desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		cost_desired_trajectories_t& costDesiredTrajectories) {

	solverPtr_->swapCostDesiredTrajectories(costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::swapCostDesiredTrajectories(
		scalar_array_t& desiredTimeTrajectory,
		dynamic_vector_array_t& desiredStateTrajectory,
		dynamic_vector_array_t& desiredInputTrajectory) {

	solverPtr_->swapCostDesiredTrajectories(desiredTimeTrajectory,
			desiredStateTrajectory, desiredInputTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller_ptr_array_t const *
MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getOptimizedControllerPtr() const {

	return optimizedControllersStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getOptimizedTrajectoriesPtr(
		const scalar_array2_t*& optimizedTimeTrajectoriesStockPtr,
		const state_vector_array2_t*& optimizedStateTrajectoriesStockPtr,
		const input_vector_array2_t*& optimizedInputTrajectoriesStockPtr) const {

	optimizedTimeTrajectoriesStockPtr  = optimizedTimeTrajectoriesStockPtr_;
	optimizedStateTrajectoriesStockPtr = optimizedStateTrajectoriesStockPtr_;
	optimizedInputTrajectoriesStockPtr = optimizedInputTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_Settings& MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::settings() {

	return mpcSettings_;
}

} // namespace ocs2



