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
MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_ILQR()

	: BASE()
	, optimizedTimeTrajectoriesStock_(0)
	, optimizedStateTrajectoriesStock_(0)
	, optimizedInputTrajectoriesStock_(0)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_ILQR(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const scalar_array_t& partitioningTimes,
		const ILQR_Settings& ilqrSettings /* = ILQR_Settings()*/,
		const MPC_Settings& mpcSettings /* = MPC_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /* = nullptr*/,
		const mode_sequence_template_t* modeSequenceTemplatePtr /* = nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: BASE(partitioningTimes, mpcSettings)
	, optimizedTimeTrajectoriesStock_(0)
	, optimizedStateTrajectoriesStock_(0)
	, optimizedInputTrajectoriesStock_(0)

{
	// ILQR
	if (ilqrSettings.ddpSettings_.useMultiThreading_) {
		ilqrPtr_.reset( new ilqr_mp_t(
				systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
				ilqrSettings, logicRulesPtr, heuristicsFunctionPtr) );
	} else {
		ilqrPtr_.reset( new ilqr_t(
				systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr, operatingTrajectoriesPtr,
				ilqrSettings, logicRulesPtr, heuristicsFunctionPtr) );
	}

	// set base solver's pointer
	BASE::setBaseSolverPtr(ilqrPtr_.get());

	// set mode sequence template
	if (modeSequenceTemplatePtr) {
		ilqrPtr_->getLogicRulesPtr()->setModeSequenceTemplate(*modeSequenceTemplatePtr);

		if (mpcSettings.recedingHorizon_) {
			const scalar_t timeHorizon = BASE::initPartitioningTimes_.back() - BASE::initPartitioningTimes_.front();
			ilqrPtr_->getLogicRulesPtr()->insertInternalModeSequenceTemplate(timeHorizon, 2.0*timeHorizon);
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
ILQR_Settings& MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::ilqrSettings() {

	return ilqrPtr_->settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::ilqr_base_t*
MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSolverPtr() {

	return ilqrPtr_.get();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ILQR<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array2_t*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr,
		const controller_ptr_array_t*& controllerStockPtr)  {

	//*****************************************************************************************
	// cost goal check
	//*****************************************************************************************
	if (BASE::initRun_==true && ilqrPtr_->costDesiredTrajectoriesUpdated()==false) {
		std::cerr << "### WARNING: The initial desired trajectories are not set. "
				"This may cause undefined behavior. Use the MPC_ILQR::setCostDesiredTrajectories() "
				"method to provide appropriate goal trajectories." << std::endl;
	}

	//*****************************************************************************************
	// updating real-time iteration settings
	//*****************************************************************************************
	// number of iterations
	if (BASE::initRun_==true /*|| ilqrPtr_->getController().at(BASE::finalActivePartitionIndex_).empty()==true*/) {
		ilqrPtr_->ddpSettings().maxNumIterations_  = BASE::mpcSettings_.initMaxNumIterations_;
		ilqrPtr_->ddpSettings().maxLearningRate_ = BASE::mpcSettings_.initMaxLearningRate_;
		ilqrPtr_->ddpSettings().minLearningRate_ = BASE::mpcSettings_.initMinLearningRate_;
	} else {
		ilqrPtr_->ddpSettings().maxNumIterations_  = BASE::mpcSettings_.runtimeMaxNumIterations_;
		ilqrPtr_->ddpSettings().maxLearningRate_ = BASE::mpcSettings_.runtimeMaxLearningRate_;
		ilqrPtr_->ddpSettings().minLearningRate_ = BASE::mpcSettings_.runtimeMinLearningRate_;
	}

	// use parallel Riccati solver at each call of realtime-iteration ILQR
	if (BASE::initRun_==false) {
		if (BASE::mpcSettings_.useParallelRiccatiSolver_==true && BASE::mpcSettings_.recedingHorizon_==true)
			ilqrPtr_->useParallelRiccatiSolverFromInitItr(true);
		else
			ilqrPtr_->useParallelRiccatiSolverFromInitItr(false);
	} else {
		ilqrPtr_->useParallelRiccatiSolverFromInitItr(false);
	}

	//*****************************************************************************************
	// calculate controller
	//*****************************************************************************************
	if (BASE::mpcSettings_.coldStart_==true || BASE::initRun_==true) {

		if (BASE::mpcSettings_.debugPrint_)
			std::cerr << "### Using cold initialization." << std::endl;

		ilqrPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_);

	} else {
		ilqrPtr_->run(initTime, initState, finalTime, BASE::partitioningTimes_,
				typename ilqr_base_t::controller_ptr_array_t());
	}

	//*****************************************************************************************
	// Get optimized outputs
	//*****************************************************************************************
	// swap the optimized trajectories
	optimizedTimeTrajectoriesStock_.clear();
	optimizedStateTrajectoriesStock_.clear();
	optimizedInputTrajectoriesStock_.clear();
	ilqrPtr_->swapNominalTrajectories(
			optimizedTimeTrajectoriesStock_,
			optimizedStateTrajectoriesStock_,
			optimizedInputTrajectoriesStock_);
	timeTrajectoriesStockPtr  = &optimizedTimeTrajectoriesStock_;
	stateTrajectoriesStockPtr = &optimizedStateTrajectoriesStock_;
	inputTrajectoriesStockPtr = &optimizedInputTrajectoriesStock_;

	// get the optimal controller
	ilqrPtr_->getControllerPtr(controllerStockPtr);

}

} // namespace ocs2

