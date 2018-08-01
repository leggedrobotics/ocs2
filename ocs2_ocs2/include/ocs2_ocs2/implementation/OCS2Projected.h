/*
 * OCS2Projected.h
 *
 *  Created on: Jul 21, 2016
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::OCS2Projected(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const SLQ_Settings& slqSettings /*= SLQ_Settings()*/,
		const LOGIC_RULES_T* logicRulesPtr /*= nullptr*/,
		const cost_function_base_t* heuristicsFunctionPtr /*= nullptr*/)

	: numSubsystems_(0)
//	, subsystemDynamicsPtr_(subsystemDynamicsPtr)
//	, subsystemDerivativesPtr_(subsystemDerivativesPtr)
//	, subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr)
	, slqSettings_(slqSettings)
{

	// NLP optimizer options
	BASE::nlpSettings().displayGradientDescent_ = slqSettings_.displayGradientDescent_;
	BASE::nlpSettings().maxIterations_ 	  = slqSettings_.maxIterationGradientDescent_;
	BASE::nlpSettings().minRelCost_    	  = slqSettings_.acceptableTolGradientDescent_;
	BASE::nlpSettings().maxLearningRate_  = slqSettings_.maxLearningRateNLP_;
	BASE::nlpSettings().minLearningRate_  = slqSettings_.minLearningRateNLP_;
	BASE::nlpSettings().minDisToBoundary_ = slqSettings_.minEventTimeDifference_;
	BASE::nlpSettings().useAscendingLineSearchNLP_ = slqSettings_.useAscendingLineSearchNLP_;
	adjustOptions();

	// SLQ solvers
	slqSolverPtrs_.resize(numLineSearch_+1);
	gslqSolverPtrs_.resize(numLineSearch_+1);
	for (size_t i=0; i<slqSolverPtrs_.size(); i++) {
		if (slqSettings_.useMultiThreading_==true) {
			slqSolverPtrs_[i] = slq_base_ptr_t(
					new slq_mp_t(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr,
							operatingTrajectoriesPtr, slqSettings, logicRulesPtr, heuristicsFunctionPtr) );
		} else {
			slqSolverPtrs_[i] = slq_base_ptr_t(
					new slq_t(systemDynamicsPtr, systemDerivativesPtr, systemConstraintsPtr, costFunctionPtr,
							operatingTrajectoriesPtr, slqSettings, logicRulesPtr, heuristicsFunctionPtr) );
		}

		gslqSolverPtrs_[i] = gslq_ptr_t(new gslq_t(*slqSolverPtrs_[i]));

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostFunction(
		scalar_t& costFunction) const  {

	costFunction = optimizedTotalCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostFunctionDerivative(
		dynamic_vector_t& costFuntionDerivative) const {

	costFuntionDerivative = costFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getEventTimes(
		scalar_array_t& eventTimes) const {

	eventTimes = optimizedEventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getController(
		controller_array_t& optimizedControllersStock)  const  {

	optimizedControllersStock = optimizedControllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getControllerPtr(
		std::shared_ptr<controller_array_t>& controllersStock) const  {

	controllersStock = std::make_shared<controller_array_t>(optimizedControllersStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller_t&
	OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller(size_t index) const  {

	return optimizedControllersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectories(
		std::vector<scalar_array_t>& optimizedTimeTrajectoriesStock,
		state_vector_array2_t& optimizedStateTrajectoriesStock,
		input_vector_array2_t& optimizedInputTrajectoriesStock) const {

	optimizedTimeTrajectoriesStock   = optimizedTimeTrajectoriesStock_;
	optimizedStateTrajectoriesStock  = optimizedStateTrajectoriesStock_;
	optimizedInputTrajectoriesStock  = optimizedInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_Settings& OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::slqSettings() {

	return slqSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectoriesPtr(
		std::shared_ptr<std::vector<scalar_array_t>>& optimizedTimeTrajectoriesStockPtr,
		std::shared_ptr<state_vector_array2_t>& optimizedStateTrajectoriesStockPtr,
		std::shared_ptr<input_vector_array2_t>& optimizedInputTrajectoriesStockPtr) const {

	optimizedTimeTrajectoriesStockPtr  = std::make_shared<std::vector<scalar_array_t>>(optimizedTimeTrajectoriesStock_);
	optimizedStateTrajectoriesStockPtr = std::make_shared<state_vector_array2_t>(optimizedStateTrajectoriesStock_);
	optimizedInputTrajectoriesStockPtr = std::make_shared<input_vector_array2_t>(optimizedInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findNearestController(
		const dynamic_vector_t& enquiryParameter) const {

	if (parameterBag_.size()==0)
		throw  std::runtime_error("controllerStock bag is empty.");

	// evaluating distance
	std::vector<scalar_t> distance(parameterBag_.size());
	for (size_t i=0; i<parameterBag_.size(); i++)
		distance[i] = (parameterBag_[i]-enquiryParameter).squaredNorm();

	// min index
	auto it = std::min_element(distance.begin(), distance.end());
	size_t index = std::distance(distance.begin(), it);

	return index;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateLinearEqualityConstraint(
		dynamic_matrix_t& Am,
		dynamic_vector_t& Bv) {

	Am = dynamic_matrix_t::Zero(BASE::numParameters()+1, BASE::numParameters());
	for (size_t i=0; i<BASE::numParameters()+1; i++) {

		if (i < BASE::numParameters())
			Am(i,i)  = -1.0;

		if (i > 0)
			Am(i,i-1) = 1.0;
	}

	Bv = dynamic_vector_t::Zero(BASE::numParameters()+1);
	Bv(0) = initEventTimes_.front();
	Bv(BASE::numParameters()) = -initEventTimes_.back();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateGradient(
		const size_t& id,
		const dynamic_vector_t& parameters,
		dynamic_vector_t& gradient) {

	gslqSolverPtrs_[id]->run();
	gslqSolverPtrs_[id]->getCostFuntionDerivative(costFuntionDerivative_);
	gradient = costFuntionDerivative_;

	return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCost(
		const size_t& id,
		const dynamic_vector_t& parameters,
		scalar_t& cost) {

	// switching time vector
	scalar_array_t eventTimes(BASE::numParameters());
	for (size_t j=0; j<BASE::numParameters(); j++)
		eventTimes[j] = parameters(j);

	// TODO
	LOGIC_RULES_T logicRules(eventTimes);
	slqSolverPtrs_[id]->setLogicRules(logicRules);

	// initial controller
	if (slqSettings_.warmStartGSLQ_==true && parameterBag_.size()>0) {

		// find most similar controller based on the parameter
		size_t warmStartIndex = findNearestController(parameters);

		// run SLQ
		try {
			slqSolverPtrs_[id]->run(initTime_, initState_, finalTime_, partitioningTimes_,
					controllersStockBag_[warmStartIndex]);
		}
		catch (const std::exception& e) {

			std::cerr << "\t     exception: " << e.what();
			return false;
		}

	} else {

		// run SLQ
		try {
			slqSolverPtrs_[id]->run(initTime_, initState_, finalTime_, partitioningTimes_);
		}
		catch (const std::exception& e) {

			std::cerr << "\t     exception: " << e.what();
			return false;
		}
	}


	scalar_t unconstraintCost, constraintISE1, constraintISE2;
	slqSolverPtrs_[id]->getPerformanceIndeces(unconstraintCost, constraintISE1, constraintISE2);
	cost = unconstraintCost;

	// display
	if (BASE::nlpSettings().displayGradientDescent_) {
		std::cerr << "\t     constraintISE1: " << constraintISE1 <<
			"\t     constraintISE2: " << constraintISE2 <<
			"\t#Iterations: " << slqSolverPtrs_[id]->getNumIterations() << std::endl;
	}

	// saving solution in the bag
	saveToBag(id, parameters);

	// status is false if the constraintISE is higher than minAbsConstraint1RMSE_
	bool status1 = (constraintISE1 <= slqSettings_.minAbsConstraint1ISE_) ? true : false;
	bool status2 = (constraintISE2 <= slqSettings_.minAbsConstraint1ISE_) ? true : false;

	return status1 && status2;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::saveToBag(
		size_t id,
		const dynamic_vector_t& parameters)  {

	// get the nominal trajectories
	const std::vector<scalar_array_t>* timeTrajectoriesStockPtr;
	const state_vector_array2_t* stateTrajectoriesStockPtr;
	const input_vector_array2_t* inputTrajectoriesStockPtr;
	slqSolverPtrs_[id]->getNominalTrajectoriesPtr(
			timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr);

	// get the optimized controller
	controller_array_t controllersStock(numPartitions_);
	slqSolverPtrs_[id]->swapController(controllersStock);

	// changing the controller structure to tracking controller
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >   nominalStateFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > nominalInputFunc;
	for (size_t i=0; i<numPartitions_; i++) {

		nominalStateFunc.setTimeStamp( &(timeTrajectoriesStockPtr->at(i)) );
		nominalStateFunc.setData( &(stateTrajectoriesStockPtr->at(i)) );

		nominalInputFunc.setTimeStamp( &(timeTrajectoriesStockPtr->at(i)) );
		nominalInputFunc.setData( &(inputTrajectoriesStockPtr->at(i)) );

		controllersStock[i].deltaUff_.resize(controllersStock[i].time_.size());
		for (size_t k=0; k<controllersStock[i].time_.size(); k++) {

			const scalar_t& time = controllersStock[i].time_[k];
			state_vector_t nominalState;
			nominalStateFunc.interpolate(time, nominalState);
			size_t greatestLessTimeStampIndex = nominalStateFunc.getGreatestLessTimeStampIndex();
			input_vector_t nominalInput;
			nominalInputFunc.interpolate(time, nominalInput, greatestLessTimeStampIndex);

			controllersStock[i].uff_[k] = -controllersStock[i].k_[k]*nominalState;
			controllersStock[i].deltaUff_[k] = nominalInput;

		} // end of k loop
	}  // end of i loop

	// save the parameter and controller in the Bag
	parameterBag_.push_back(parameters);
	controllersStockBag_.push_back(controllersStock);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateInitialController(
		const state_vector_t& initState,
		const scalar_array_t& eventTimes,
		controller_array_t& controllersStock) {
//
//
//	// using GLQP for coldStart controller
//	controller_array_t coldStartControllersStock(numPartitions_);
//
//	// calculate the coldStart controllers' cost
//	std::vector<scalar_array_t> timeTrajectoriesStock(numPartitions_);
//	state_vector_array2_t stateTrajectoriesStock(numPartitions_);
//	input_vector_array2_t inputTrajectoriesStock(numPartitions_);
//	scalar_t coldStartTotalCost = std::numeric_limits<scalar_t>::max();
//	try {
//		rollout(initTime_, initState_, finalTime_, switchingTimes, coldStartControllersStock,
//				timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
//		calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock, coldStartTotalCost);
//	}
//	catch (const std::exception& e){
//		std::cerr << "\t     Cold start failed!" << std::endl;
//	}
//
//
//	controller_array_t warmStartControllersStock(numPartitions_);
//	scalar_t warmStartTotalCost = std::numeric_limits<scalar_t>::max();
//	size_t warmStartIndex;
//	if (slqSettings_.warmStartGSLQ_==true && parameterBag_.size()>0) {
//
//		// find most similar controller based on the parameter
//		const dynamic_vector_t parameters = Eigen::Map<const dynamic_vector_t>(&switchingTimes[1], BASE::numParameters());
//		warmStartIndex = findNearestController(parameters);
//
//		warmStartControllersStock = controllersStockBag_[warmStartIndex];
//		// scaling the controller time to the current switchingTimes
//		for (size_t i=0; i<numPartitions_; i++) {
//			scalar_t scale = (switchingTimes[i+1]-switchingTimes[i]) / (warmStartControllersStock[i].time_.back()-warmStartControllersStock[i].time_.front());
//			for (size_t k=0; k<warmStartControllersStock[i].time_.size(); k++) {
//				warmStartControllersStock[i].time_[k] = switchingTimes[i] + scale*(warmStartControllersStock[i].time_[k]-warmStartControllersStock[i].time_.front());
//				/*previously used by farbod to scale velocities with switching times: */
//				warmStartControllersStock[i].uff_[k].head(12) += warmStartControllersStock[i].deltaUff_[k].head(12);
//				warmStartControllersStock[i].uff_[k].tail(12) += warmStartControllersStock[i].deltaUff_[k].tail(12) / scale;
//			} // end of k loop
//		}  // end of i loop
//
//		// calculate the warmStart controllers' cost
//		std::vector<scalar_array_t> timeTrajectoriesStock(numPartitions_);
//		state_vector_array2_t stateTrajectoriesStock(numPartitions_);
//		input_vector_array2_t inputTrajectoriesStock(numPartitions_);
//		try {
//			rollout(initTime_, initState_, finalTime_, switchingTimes, warmStartControllersStock,
//					timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
//			calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock,
//					warmStartTotalCost);
//		} catch (const std::exception& e) {}
//
//	}
//
//	// choose which controller to use based on the cost function
//	if (slqSettings_.warmStartGSLQ_==true && warmStartTotalCost<=coldStartTotalCost) {
//		controllersStock.swap(warmStartControllersStock);
//		std::cerr << "\t     Warm start!" << std::endl;
//		if (BASE::nlpSettings().displayGradientDescent_)
//			std::cerr << "\t     IndexFromBack: " << static_cast<int>(warmStartIndex-(parameterBag_.size()-1))
//						<< "\t#parameters: " << parameterBag_[warmStartIndex].transpose().format(CleanFmtDisplay_) << std::endl;
//	} else {
//		controllersStock.swap(coldStartControllersStock);
//		if (BASE::nlpSettings().displayGradientDescent_)  std::cerr << "\t     Cold start!" << std::endl;
//	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rollout(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& switchingTimes,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		input_vector_array2_t& inputTrajectoriesStock)  {

//	if (controllersStock.size() != numPartitions_)
//		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");
//
//	timeTrajectoriesStock.resize(numPartitions_);
//	stateTrajectoriesStock.resize(numPartitions_);
//	inputTrajectoriesStock.resize(numPartitions_);
//
//	// finding the active subsystem index at initTime
//	size_t initActiveSubsystemIndex = slq_base_t::findActiveSubsystemIndex(switchingTimes, initTime);
//	// finding the active subsystem index at initTime
//	size_t finalActiveSubsystemIndex = slq_base_t::findActiveSubsystemIndex(switchingTimes, finalTime);
//
//	scalar_t t0 = initTime;
//	state_vector_t x0 = initState;
//	scalar_t tf;
//	for (int i=0; i<numPartitions_; i++) {
//
//		// for subsystems before the initial time
//		if (i<initActiveSubsystemIndex  ||  i>finalActiveSubsystemIndex) {
//			timeTrajectoriesStock[i].clear();
//			stateTrajectoriesStock[i].clear();
//			inputTrajectoriesStock[i].clear();
//			continue;
//		}
//
//		timeTrajectoriesStock[i].clear();
//		stateTrajectoriesStock[i].clear();
//
//		// max number of steps of integration
//		size_t maxNumSteps = slqSettings_.maxNumStepsPerSecond_ * std::max(1.0, switchingTimes[i+1]-t0);
//
//		// initialize subsystem i
//		subsystemDynamicsPtrStock_[i]->initializeModel(systemStockIndexes_, switchingTimes, x0, i, "GSLPQ");
//
//		// final time
//		tf = (i != finalActiveSubsystemIndex) ? switchingTimes[i+1] : finalTime;
//
//		// set controller for subsystem i
//		if (controllersStock[i].time_.empty()==false) {
//			subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);
//
//		} else {
//
//			if (slqSettings_.dispayGSLQ_)  std::cout << "LQP controller is used at period: [" << t0 << ", " << tf << "]" << std::endl;
//
//			controller_t lqpPolicy = lqpControllersStock_[systemStockIndexes_[i]];
//			scalar_t timeShift = t0-lqpPolicy.time_[0];
//			for (size_t k=0; k<lqpPolicy.time_.size(); k++)
//				lqpPolicy.time_[k] += timeShift;
//
//			subsystemDynamicsPtrStock_[i]->setController(lqpPolicy);
//		}
//
//		// simulate subsystem i
//		subsystemSimulatorsStockPtr_[i]->integrate(x0, t0, tf,
//				stateTrajectoriesStock[i], timeTrajectoriesStock[i],
//				1e-3, slqSettings_.AbsTolODE_, slqSettings_.RelTolODE_, maxNumSteps);
//
//		if (stateTrajectoriesStock[i].back() != stateTrajectoriesStock[i].back())
//			throw std::runtime_error("System became unstable during the SLQ rollout.");
//
//		// compute control trajectory for subsystem i
//		inputTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
//		for (int k=0; k<timeTrajectoriesStock[i].size(); k++) {
//			subsystemDynamicsPtrStock_[i]->computeInput(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
//		}
//
//		// reset the initial time and state
//		t0 = timeTrajectoriesStock[i].back();
//		x0 = stateTrajectoriesStock[i].back();
//	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCostFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

//	totalCost = 0.0;
//	for (int i=0; i<numPartitions_; i++) {
//
//		// integrates the intermediate cost using the trapezoidal approximation method
//		scalar_t currentIntermediateCost;
//		scalar_t nextIntermediateCost;
//		for (int k=0; k<timeTrajectoriesStock[i].size()-1; k++) {
//
//			if (k==0) {
//				subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
//				subsystemCostFunctionsPtrStock_[i]->evaluate(currentIntermediateCost);
//			} else {
//				currentIntermediateCost = nextIntermediateCost;
//			}
//
//			// feed next state and control to cost function
//			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k+1], stateTrajectoriesStock[i][k+1], inputTrajectoriesStock[i][k+1]);
//			// evaluate intermediate cost for next time step
//			subsystemCostFunctionsPtrStock_[i]->evaluate(nextIntermediateCost);
//
//			totalCost += 0.5*(currentIntermediateCost+nextIntermediateCost)*(timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);
//		}  // end of k loop
//
//		// terminal cost
//		if (i==numSubsystems_-1)  {
//			scalar_t finalCost;
//			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i].back(), stateTrajectoriesStock[i].back(), inputTrajectoriesStock[i].back());
//			subsystemCostFunctionsPtrStock_[i]->terminalCost(finalCost);
//			totalCost += finalCost;
//		}
//
//	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSolution(size_t idStar)  {

	optimizedEventTimes_ = slqSolverPtrs_[idStar]->getEventTimes();
	optimizedControllersStock_ = slqSolverPtrs_[idStar]->getController();
	optimizedTimeTrajectoriesStock_  = slqSolverPtrs_[idStar]->getNominalTimeTrajectories();
	optimizedStateTrajectoriesStock_ = slqSolverPtrs_[idStar]->getNominalStateTrajectories();
	optimizedInputTrajectoriesStock_ = slqSolverPtrs_[idStar]->getNominalInputTrajectories();
	slqSolverPtrs_[idStar]->getPerformanceIndeces(
			optimizedTotalCost_, optimizedConstraint1ISE_, optimizedConstraint2ISE_);
	slqSolverPtrs_[idStar]->getIterationsLog(
			slqIterationCost_, slqIterationISE1_, slqIterationISE2_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rewindOptimizer(
		const size_t& firstIndex, bool initRun/*=false*/) {

	// rewind SLQ solvers
	for (size_t i=0; i<slqSolverPtrs_.size(); i++)
		slqSolverPtrs_[i]->rewindOptimizer(firstIndex, initRun);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer() {

//	subsystemDynamicsPtrStock_.resize(numPartitions_);
//	subsystemCostFunctionsPtrStock_.resize(numPartitions_);
//	subsystemSimulatorsStockPtr_.resize(numPartitions_);
//
//	for (int i=0; i<systemStockIndexes_.size(); i++) {
//		subsystemDynamicsPtrStock_[i] = subsystemDynamicsPtr_[systemStockIndexes_[i]]->clone();
//		subsystemCostFunctionsPtrStock_[i] = subsystemCostFunctionsPtr_[systemStockIndexes_[i]]->clone();
//		subsystemSimulatorsStockPtr_[i] =  std::shared_ptr<ODE45<STATE_DIM>>( new ODE45<STATE_DIM>(subsystemDynamicsPtrStock_[i]) );
//	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const scalar_array_t& initEventTimes /*= std::vector<scalar_t>()*/,
		const cost_desired_trajectories_t& costDesiredTrajectories /*= cost_desired_trajectories_t()*/) {

	numSubsystems_  = initEventTimes.size()+1;
	initEventTimes_ = initEventTimes;

	setupOptimizer();

	parameterBag_.clear();
	controllersStockBag_.clear();

	initTime_ = initTime;
	initState_ = initState;
	finalTime_ = finalTime;
	numPartitions_ = partitioningTimes.size();
	partitioningTimes_ = partitioningTimes;

	// update cost's desired trajectory if it is set
	if (costDesiredTrajectories.empty()==false)
		for (size_t i=0; i<slqSolverPtrs_.size(); i++)
			slqSolverPtrs_[i]->setCostDesiredTrajectories(costDesiredTrajectories);

	for (size_t i=0; i<slqSolverPtrs_.size(); i++)
		slqSolverPtrs_[i]->settings() = slqSettings_;

	// run
	dynamic_vector_t initParameters = Eigen::Map<dynamic_vector_t>(initEventTimes_.data(), numSubsystems_-1);
	BASE::run(initParameters);

}


}  // end of ocs2 namespace
