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
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_SLQ()

	: BASE()
	, nullControllersStock_(0)
	, optimizedTimeTrajectoriesStock_(0)
	, optimizedStateTrajectoriesStock_(0)
	, optimizedInputTrajectoriesStock_(0)
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

	: BASE(partitioningTimes, mpcSettings)
	, nullControllersStock_(0)
	, optimizedTimeTrajectoriesStock_(0)
	, optimizedStateTrajectoriesStock_(0)
	, optimizedInputTrajectoriesStock_(0)

{
	nullControllersStock_.resize(BASE::numPartitions_);

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

	// set base solver's pointer
	BASE::setBaseSolverPtr(slqPtr_.get());

	// set mode sequence template
	if (modeSequenceTemplatePtr) {
		slqPtr_->getLogicRulesPtr()->setModeSequenceTemplate(*modeSequenceTemplatePtr);

		if (mpcSettings.recedingHorizon_==true) {
			const scalar_t timeHorizon = BASE::initPartitioningTimes_.back() - BASE::initPartitioningTimes_.front();
			slqPtr_->getLogicRulesPtr()->insertModeSequenceTemplate(timeHorizon, 2.0*timeHorizon);
		}
	}
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
typename MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::slq_base_t*
MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSolverPtr() {

	return slqPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_SLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr,
		const controller_array_t*& controllerStockPtr)  {

	//*****************************************************************************************
	// cost goal check
	//*****************************************************************************************
	if (BASE::initRun_==true && slqPtr_->costDesiredTrajectoriesUpdated()==false) {
		std::cerr << "### WARNING: The initial desired trajectories are not set. "
				"This may cause undefined behavior. Use the MPC_SLQ::setCostDesiredTrajectories() "
				"method to provide appropriate goal trajectories." << std::endl;
	}

	//*****************************************************************************************
	// updating real-time iteration settings
	//*****************************************************************************************
	// number of iterations
	if (BASE::initRun_==true /*|| slqPtr_->getController().at(BASE::finalActivePartitionIndex_).empty()==true*/) {
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

		slqPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_);

	} else {
		slqPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_,
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

//	//*****************************************************************************************
//	// Update logicRules if new logicRulesTemplate is set
//	//*****************************************************************************************
//	if (BASE::logicRulesTemplateUpdated_ == true) {
//
//		// set new templates
//		slqPtr_->getLogicRulesPtr()->setModeSequenceTemplate(BASE::newLogicRulesTemplate_);
//		slqPtr_->getLogicRulesPtr()->insertModeSequenceTemplate(finalTime, BASE::partitioningTimes_.back());
//		slqPtr_->getLogicRulesMachinePtr()->logicRulesUpdated();
//
//		BASE::logicRulesTemplateUpdated_ = false;
//	}

}

} // namespace ocs2

