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
		const std::vector<std::shared_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM> > >& subsystemDynamicsPtr,
		const std::vector<std::shared_ptr<DerivativesBase<STATE_DIM, INPUT_DIM> > >& subsystemDerivativesPtr,
		const std::vector<std::shared_ptr<CostFunctionBase<STATE_DIM, INPUT_DIM> > >& subsystemCostFunctionsPtr,
		const SLQ_Settings& slqSettings,
		const state_vector_array_t& stateOperatingPoints,
		const input_vector_array_t& inputOperatingPoints)

	: numSubsystems_(0)
	, subsystemDynamicsPtr_(subsystemDynamicsPtr)
	, subsystemDerivativesPtr_(subsystemDerivativesPtr)
	, subsystemCostFunctionsPtr_(subsystemCostFunctionsPtr)
	, slqSettings_(slqSettings)
{
	// NLP optimizer options
	nlpOptions_.displayGradientDescent_ = slqSettings_.displayGradientDescent_;
	nlpOptions_.maxIterations_ 	  = slqSettings_.maxIterationGradientDescent_;
	nlpOptions_.minRelCost_    	  = slqSettings_.acceptableTolGradientDescent_;
	nlpOptions_.maxLearningRate_  = slqSettings_.maxLearningRateNLP_;
	nlpOptions_.minLearningRate_  = slqSettings_.minLearningRateNLP_;
	nlpOptions_.minDisToBoundary_ = slqSettings_.minEventTimeDifference_;
	nlpOptions_.useAscendingLineSearchNLP_ = slqSettings_.useAscendingLineSearchNLP_;
	adjustOptions();

	// setting up subsystemSimulatorsStockPtr
	if (subsystemDynamicsPtr.size() != subsystemCostFunctionsPtr.size())
		throw std::runtime_error("Number of cost functions is not equal to the number of subsystems.");

	// SLQ solvers
	slqSolverPtrs_.resize(numLineSearch_+1);
	for (size_t i=0; i<slqSolverPtrs_.size(); i++)
		if (slqSettings_.useMultiThreading_==true)
			slqSolverPtrs_[i] = slq_base_ptr_t( new slq_mp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
					slqSettings, stateOperatingPoints, inputOperatingPoints) );
		else
			slqSolverPtrs_[i] = slq_base_ptr_t( new slq_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr,
					slqSettings, stateOperatingPoints, inputOperatingPoints) );

	// GSLQ solver
	gslqSolver_ = gslq_ptr_t( new gslq_t(slqSettings) );

	// LQP solver
	lqp_ptr_t lqpPtr_( new lqp_t(subsystemDynamicsPtr, subsystemDerivativesPtr, subsystemCostFunctionsPtr, stateOperatingPoints, inputOperatingPoints) );
	const size_t N = subsystemDynamicsPtr.size();
	std::vector<size_t>   systemStockIndexes(1);
	std::vector<scalar_t> switchingTimes(2);
	lqpControllersStock_.resize(N);
	for (size_t i=0; i<N; i++) {

		systemStockIndexes[0] = i;
		switchingTimes[0] = 0.0;
		switchingTimes[1] = 1.0;

		lqpPtr_->run(systemStockIndexes, switchingTimes, 0.0/*=1.0*/);
		controller_array_t local_controller(1);
		lqpPtr_->getController(local_controller);
		lqpControllersStock_[i].swap(local_controller[0]);
	} // end of i loop

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
		Eigen::VectorXd& costFuntionDerivative) const {

	costFuntionDerivative = costFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSwitchingTimes(
		scalar_array_t& switchingTimes) const {

	switchingTimes = optimizedSwitchingTimes_;
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

	optimizedTimeTrajectoriesStockPtr   = std::make_shared<std::vector<scalar_array_t>>(optimizedTimeTrajectoriesStock_);
	optimizedStateTrajectoriesStockPtr  = std::make_shared<state_vector_array2_t>(optimizedStateTrajectoriesStock_);
	optimizedInputTrajectoriesStockPtr  = std::make_shared<input_vector_array2_t>(optimizedInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findNearestController(
		const Eigen::VectorXd& enquiryParameter) const {

	if (parameterBag_.size()==0)  throw  std::runtime_error("controllerStock bag is empty.");

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
		Eigen::MatrixXd& Am,
		Eigen::VectorXd& Bv) {

	Am = Eigen::MatrixXd::Zero(numParameters_+1, numParameters_);
	for (size_t i=0; i<numParameters_+1; i++) {
		if (i<numParameters_) 	Am(i,i)  = -1.0;
		if (i>0)				Am(i,i-1) = 1.0;
	}

	Bv = Eigen::VectorXd::Zero(numParameters_+1);
	Bv(0) = initSwitchingTimes_.front();
	Bv(numParameters_) = -initSwitchingTimes_.back();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateGradient(
		const size_t& id,
		const Eigen::VectorXd& parameters,
		Eigen::VectorXd& gradient) {

	// switching time vector
	scalar_array_t switchingTimes(initSwitchingTimes_);
	for (size_t j=0; j<numParameters_; j++)
		switchingTimes[j+1] = parameters(j);

	/*
	 * solve Sequential Riccati Equations with learningRate 0.0,
	 * calculate the nominal co-state,
	 * add the deleted controller parts
	 */
	slqSolverPtrs_[id]->runExit();

	gslqSolver_->run(initTime_, initState_, finalTime_, systemStockIndexes_, switchingTimes, slqSolverPtrs_[id],
			desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
	gslqSolver_->getCostFuntionDerivative(costFuntionDerivative_);
	gradient = costFuntionDerivative_;

	return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCost(
		const size_t& id,
		const Eigen::VectorXd& parameters,
		scalar_t& cost) {

	// switching time vector
	scalar_array_t switchingTimes(initSwitchingTimes_);
	for (size_t j=0; j<numParameters_; j++)
		switchingTimes[j+1] = parameters(j);

	// initial controller
	controller_array_t controllersStock(numSubsystems_);
	calculateInitialController(initState_, switchingTimes, controllersStock);

	// run SLQ
	try {
		slqSolverPtrs_[id]->run(initTime_, initState_, finalTime_, systemStockIndexes_, switchingTimes, controllersStock,
				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
	}
	catch (const std::exception& e){
		std::cerr << "\t     exception: " << e.what();
		return false;
	}

	scalar_t unconstraintCost, constraintISE;
	slqSolverPtrs_[id]->getCostFuntion(unconstraintCost, constraintISE);
	cost = unconstraintCost;

	// display
	if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     constraintISE: " << constraintISE
			<< "\t#Iterations: " << slqSolverPtrs_[id]->getNumIterations() << std::endl;

	// saving solution in the bag
	saveToBag(id, parameters);

	// status is false if the constraintISE is higher than minAbsConstraint1RMSE_
	bool status = (constraintISE <= slqSettings_.minAbsConstraint1ISE_) ? true : false;

	return status;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::saveToBag(
		size_t id,
		const Eigen::VectorXd& parameters)  {

	// get the nominal trajectories
	std::vector<scalar_array_t> timeTrajectoriesStock(numSubsystems_);
	state_vector_array2_t   stateTrajectoriesStock(numSubsystems_);
	input_vector_array2_t inputTrajectoriesStock(numSubsystems_);
	slqSolverPtrs_[id]->getNominalTrajectories(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);

	// get the optimized controller
	controller_array_t controllersStock(numSubsystems_);
	slqSolverPtrs_[id]->getController(controllersStock);

	// changing the controller structure to tracking controller
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >   nominalStateFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > nominalInputFunc;
	for (size_t i=0; i<numSubsystems_; i++) {

		nominalStateFunc.setTimeStamp( &(timeTrajectoriesStock[i]) );
		nominalStateFunc.setData( &(stateTrajectoriesStock[i]) );

		nominalInputFunc.setTimeStamp( &(timeTrajectoriesStock[i]) );
		nominalInputFunc.setData( &(inputTrajectoriesStock[i]) );

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
		const scalar_array_t& switchingTimes,
		controller_array_t& controllersStock) {


	// using GLQP for coldStart controller
	controller_array_t coldStartControllersStock(numSubsystems_);

	// calculate the coldStart controllers' cost
	std::vector<scalar_array_t> timeTrajectoriesStock(numSubsystems_);
	state_vector_array2_t stateTrajectoriesStock(numSubsystems_);
	input_vector_array2_t inputTrajectoriesStock(numSubsystems_);
	scalar_t coldStartTotalCost = std::numeric_limits<scalar_t>::max();
	try {
		rollout(initTime_, initState_, finalTime_, switchingTimes, coldStartControllersStock,
				timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
		calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock, coldStartTotalCost);
	}
	catch (const std::exception& e){
		std::cerr << "\t     Cold start failed!" << std::endl;
	}


	controller_array_t warmStartControllersStock(numSubsystems_);
	scalar_t warmStartTotalCost = std::numeric_limits<scalar_t>::max();
	size_t warmStartIndex;
	if (slqSettings_.warmStartGSLQ_==true && parameterBag_.size()>0) {

		// find most similar controller based on the parameter
		const Eigen::VectorXd parameters = Eigen::Map<const Eigen::VectorXd>(&switchingTimes[1], numParameters_);
		warmStartIndex = findNearestController(parameters);

		warmStartControllersStock = controllersStockBag_[warmStartIndex];
		// scaling the controller time to the current switchingTimes
		for (size_t i=0; i<numSubsystems_; i++) {
			scalar_t scale = (switchingTimes[i+1]-switchingTimes[i]) / (warmStartControllersStock[i].time_.back()-warmStartControllersStock[i].time_.front());
			for (size_t k=0; k<warmStartControllersStock[i].time_.size(); k++) {
				warmStartControllersStock[i].time_[k] = switchingTimes[i] + scale*(warmStartControllersStock[i].time_[k]-warmStartControllersStock[i].time_.front());
				/*previously used by farbod to scale velocities with switching times: */
				warmStartControllersStock[i].uff_[k].head(12) += warmStartControllersStock[i].deltaUff_[k].head(12);
				warmStartControllersStock[i].uff_[k].tail(12) += warmStartControllersStock[i].deltaUff_[k].tail(12) / scale;
			} // end of k loop
		}  // end of i loop

		// calculate the warmStart controllers' cost
		std::vector<scalar_array_t> timeTrajectoriesStock(numSubsystems_);
		state_vector_array2_t stateTrajectoriesStock(numSubsystems_);
		input_vector_array2_t inputTrajectoriesStock(numSubsystems_);
		try {
			rollout(initTime_, initState_, finalTime_, switchingTimes, warmStartControllersStock,
					timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
			calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock,
					warmStartTotalCost);
		} catch (const std::exception& e) {}

	}

	// choose which controller to use based on the cost function
	if (slqSettings_.warmStartGSLQ_==true && warmStartTotalCost<=coldStartTotalCost) {
		controllersStock.swap(warmStartControllersStock);
		std::cerr << "\t     Warm start!" << std::endl;
		if (nlpOptions_.displayGradientDescent_)
			std::cerr << "\t     IndexFromBack: " << static_cast<int>(warmStartIndex-(parameterBag_.size()-1))
						<< "\t#parameters: " << parameterBag_[warmStartIndex].transpose().format(CleanFmtDisplay_) << std::endl;
	} else {
		controllersStock.swap(coldStartControllersStock);
		if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     Cold start!" << std::endl;
	}

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

	if (controllersStock.size() != numSubsystems_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(numSubsystems_);
	stateTrajectoriesStock.resize(numSubsystems_);
	inputTrajectoriesStock.resize(numSubsystems_);

	// finding the active subsystem index at initTime
	size_t initActiveSubsystemIndex = slq_base_t::findActiveSubsystemIndex(switchingTimes, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = slq_base_t::findActiveSubsystemIndex(switchingTimes, finalTime);

	scalar_t t0 = initTime;
	state_vector_t x0 = initState;
	scalar_t tf;
	for (int i=0; i<numSubsystems_; i++) {

		// for subsystems before the initial time
		if (i<initActiveSubsystemIndex  ||  i>finalActiveSubsystemIndex) {
			timeTrajectoriesStock[i].clear();
			stateTrajectoriesStock[i].clear();
			inputTrajectoriesStock[i].clear();
			continue;
		}

		timeTrajectoriesStock[i].clear();
		stateTrajectoriesStock[i].clear();

		// max number of steps of integration
		size_t maxNumSteps = slqSettings_.maxNumStepsPerSecond_ * std::max(1.0, switchingTimes[i+1]-t0);

		// initialize subsystem i
		subsystemDynamicsPtrStock_[i]->initializeModel(systemStockIndexes_, switchingTimes, x0, i, "GSLPQ");

		// final time
		tf = (i != finalActiveSubsystemIndex) ? switchingTimes[i+1] : finalTime;

		// set controller for subsystem i
		if (controllersStock[i].time_.empty()==false) {
			subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);

		} else {

			if (slqSettings_.dispayGSLQ_)  std::cout << "LQP controller is used at period: [" << t0 << ", " << tf << "]" << std::endl;

			controller_t lqpPolicy = lqpControllersStock_[systemStockIndexes_[i]];
			scalar_t timeShift = t0-lqpPolicy.time_[0];
			for (size_t k=0; k<lqpPolicy.time_.size(); k++)
				lqpPolicy.time_[k] += timeShift;

			subsystemDynamicsPtrStock_[i]->setController(lqpPolicy);
		}

		// simulate subsystem i
		subsystemSimulatorsStockPtr_[i]->integrate(x0, t0, tf,
				stateTrajectoriesStock[i], timeTrajectoriesStock[i],
				1e-3, slqSettings_.AbsTolODE_, slqSettings_.RelTolODE_, maxNumSteps);

		if (stateTrajectoriesStock[i].back() != stateTrajectoriesStock[i].back())
			throw std::runtime_error("System became unstable during the SLQ rollout.");

		// compute control trajectory for subsystem i
		inputTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
		for (int k=0; k<timeTrajectoriesStock[i].size(); k++) {
			subsystemDynamicsPtrStock_[i]->computeInput(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
		}

		// reset the initial time and state
		t0 = timeTrajectoriesStock[i].back();
		x0 = stateTrajectoriesStock[i].back();
	}
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

	totalCost = 0.0;
	for (int i=0; i<numSubsystems_; i++) {

		// integrates the intermediate cost using the trapezoidal approximation method
		scalar_t currentIntermediateCost;
		scalar_t nextIntermediateCost;
		for (int k=0; k<timeTrajectoriesStock[i].size()-1; k++) {

			if (k==0) {
				subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
				subsystemCostFunctionsPtrStock_[i]->evaluate(currentIntermediateCost);
			} else {
				currentIntermediateCost = nextIntermediateCost;
			}

			// feed next state and control to cost function
			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i][k+1], stateTrajectoriesStock[i][k+1], inputTrajectoriesStock[i][k+1]);
			// evaluate intermediate cost for next time step
			subsystemCostFunctionsPtrStock_[i]->evaluate(nextIntermediateCost);

			totalCost += 0.5*(currentIntermediateCost+nextIntermediateCost)*(timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);
		}  // end of k loop

		// terminal cost
		if (i==numSubsystems_-1)  {
			scalar_t finalCost;
			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i].back(), stateTrajectoriesStock[i].back(), inputTrajectoriesStock[i].back());
			subsystemCostFunctionsPtrStock_[i]->terminalCost(finalCost);
			totalCost += finalCost;
		}

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getSolution(size_t idStar)  {

	slqSolverPtrs_[idStar]->getCostFuntion(optimizedTotalCost_, optimizedConstraintISE_);
	slqSolverPtrs_[idStar]->getSwitchingTimes(optimizedSwitchingTimes_);
	slqSolverPtrs_[idStar]->getController(optimizedControllersStock_);
	slqSolverPtrs_[idStar]->getNominalTrajectories(optimizedTimeTrajectoriesStock_, optimizedStateTrajectoriesStock_, optimizedInputTrajectoriesStock_);
	slqSolverPtrs_[idStar]->getIterationsLog(slqIterationCost_, slqIterationISE1_);
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
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setUseDisjointRiccati(
		bool useDisjointRiccati) {

	for (size_t i=0; i<slqSolverPtrs_.size(); i++)
		slqSolverPtrs_[i]->setUseDisjointRiccati(useDisjointRiccati);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer() {

	if (subsystemDynamicsPtr_.size()-1 < *std::max_element(systemStockIndexes_.begin(), systemStockIndexes_.end()))
		throw std::runtime_error("systemStockIndex points to non-existing subsystem");

	subsystemDynamicsPtrStock_.resize(numSubsystems_);
	subsystemCostFunctionsPtrStock_.resize(numSubsystems_);
	subsystemSimulatorsStockPtr_.resize(numSubsystems_);

	for (int i=0; i<systemStockIndexes_.size(); i++) {
		subsystemDynamicsPtrStock_[i] = subsystemDynamicsPtr_[systemStockIndexes_[i]]->clone();
		subsystemCostFunctionsPtrStock_[i] = subsystemCostFunctionsPtr_[systemStockIndexes_[i]]->clone();
		subsystemSimulatorsStockPtr_[i] =  std::shared_ptr<ODE45<STATE_DIM>>( new ODE45<STATE_DIM>(subsystemDynamicsPtrStock_[i]) );
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void OCS2Projected<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const std::vector<size_t>& systemStockIndexes/*=std::vector<size_t>()*/,
		const std::vector<scalar_t>& switchingTimes/*=std::vector<scalar_t>()*/,
		const controller_array_t& controllersStock/*=controller_array_t()*/,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock/*=scalar_array2_t()*/,
		const state_vector_array2_t& desiredStateTrajectoriesStock/*=state_vector_array2_t()*/) {

	// use systemStockIndexes_ if no systemStockIndexes is given
	if (systemStockIndexes.empty()==false) {
		if (systemStockIndexes_ != systemStockIndexes) {
			numSubsystems_ = systemStockIndexes.size();
			systemStockIndexes_ = systemStockIndexes;
			setupOptimizer();
			parameterBag_.clear();
			controllersStockBag_.clear();
		}
	} else {
		if (systemStockIndexes_.empty()==true)
			throw std::runtime_error("systemStockIndexes should be provided since internal systemStockIndexes is empty.");
	}

	// use switchingTimes if no switchingTimes is given
	if (switchingTimes.empty()==false) {
		if (switchingTimes.size() != numSubsystems_+1)
			throw std::runtime_error("Number of switching times should be one plus the number of subsystems.");
		initSwitchingTimes_ = switchingTimes;
		parameterBag_.clear();
		controllersStockBag_.clear();
	} else {
		if (initSwitchingTimes_.size() != numSubsystems_+1)
			throw std::runtime_error("switchingTimes should be provided since internal one is not compatible with systemStockIndexes.");
	}

	initTime_ = initTime;
	initState_ = initState;
	finalTime_ = finalTime;
	desiredTimeTrajectoriesStock_ = desiredTimeTrajectoriesStock;
	desiredStateTrajectoriesStock_ = desiredStateTrajectoriesStock;

	for (size_t i=0; i<slqSolverPtrs_.size(); i++)
		slqSolverPtrs_[i]->slqSettings() = slqSettings_;

	// run
	Eigen::VectorXd initParameters = Eigen::Map<Eigen::VectorXd>(initSwitchingTimes_.data()+1, numSubsystems_-1);
	GradientDescent::run(initParameters);

}


}  // end of ocs2 namespace
