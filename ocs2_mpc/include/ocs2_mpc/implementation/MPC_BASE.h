/*
 * MPC_BASE.hpp
 *
 *  Created on: 11.01.2017
 *      Author: Farbod Farshidian
 */


namespace ocs2{

using namespace std::chrono;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_BASE(
		const MPC_Settings &mpcSettings /*= MPC_Settings()*/)

	: mpcSettings_(mpcSettings)
	, initRun_(true)
	, logicRulesTemplateUpdated_(false)
	, optimizedControllersStockPtr_(nullptr)
	, optimizedTimeTrajectoriesStockPtr_(nullptr)
	, optimizedStateTrajectoriesStockPtr_(nullptr)
	, optimizedInputTrajectoriesStockPtr_(nullptr)
	, measuredRuntimeMS_(0)
{}

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

		if (initRun_==true)
			throw std::runtime_error("The initial time is greater than the planning time in the first run.");

		std::cerr << std::endl << "#####################################################";
		std::cerr << std::endl << "#####################################################";
		std::cerr << std::endl << "#####################################################" << std::endl;
		std::cerr << "### MPC is called at time:  " << currentTime << " [s]." << std::endl;
		std::cerr << "WARNING: The MPC time-horizon is smaller than the MPC starting time." << std::endl;
		std::cerr << "currentTime: " << currentTime << "\t Controller finalTime: " << getFinalTime() << std::endl;

		return false;
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
	 * Calculate controller
	 ******************************************************************************************/
	// calculate the MPC controller
	scalar_t initTime = currentTime;
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
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getOptimizedControllerPtr(
		const controller_array_t*& optimizedControllersStockPtr) const {

	optimizedControllersStockPtr = optimizedControllersStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getOptimizedTrajectoriesPtr(
			const std::vector<scalar_array_t>*& optimizedTimeTrajectoriesStockPtr,
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setNewLogicRulesTemplate(
			const typename LOGIC_RULES_T::logic_template_type& newLogicRulesTemplate) {

	logicRulesTemplateUpdated_ = true;
	newLogicRulesTemplate_ = newLogicRulesTemplate;
}

} // namespace ocs2



