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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getCostFunction(scalar_t& costFunction) const  {
	costFunction = optimizedTotalCost_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getCostFunctionDerivative(
		Eigen::VectorXd& costFuntionDerivative) const {
	costFuntionDerivative = costFuntionDerivative_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getSwitchingTimes(
		scalar_array_t& switchingTimes) const {
	switchingTimes = optimizedSwitchingTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getController(controller_array_t& optimizedControllersStock)  const  {
	optimizedControllersStock = optimizedControllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const  {
	controllersStock = std::make_shared<controller_array_t>(optimizedControllersStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename OCS2Projected<STATE_DIM, INPUT_DIM>::controller_t& OCS2Projected<STATE_DIM, INPUT_DIM>::controller(size_t index) const  {
	return optimizedControllersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getNominalTrajectories(std::vector<scalar_array_t>& optimizedTimeTrajectoriesStock,
		state_vector_array2_t& optimizedStateTrajectoriesStock,
		input_vector_array2_t& optimizedInputTrajectoriesStock) const {

	optimizedTimeTrajectoriesStock   = optimizedTimeTrajectoriesStock_;
	optimizedStateTrajectoriesStock  = optimizedStateTrajectoriesStock_;
	optimizedInputTrajectoriesStock  = optimizedInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
typename OCS2Projected<STATE_DIM, INPUT_DIM>::Options_t& OCS2Projected<STATE_DIM, INPUT_DIM>::options() {
	return options_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& optimizedTimeTrajectoriesStockPtr,
		std::shared_ptr<state_vector_array2_t>& optimizedStateTrajectoriesStockPtr,
		std::shared_ptr<input_vector_array2_t>& optimizedInputTrajectoriesStockPtr) const {

	optimizedTimeTrajectoriesStockPtr   = std::make_shared<std::vector<scalar_array_t>>(optimizedTimeTrajectoriesStock_);
	optimizedStateTrajectoriesStockPtr  = std::make_shared<state_vector_array2_t>(optimizedStateTrajectoriesStock_);
	optimizedInputTrajectoriesStockPtr  = std::make_shared<input_vector_array2_t>(optimizedInputTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t OCS2Projected<STATE_DIM, INPUT_DIM>::findNearestController(const Eigen::VectorXd& enquiryParameter) const  {

	if (parameterBag_.size()==0)  throw  std::runtime_error("controllerStock bag is empty.");

	// evaluating distance
	std::vector<double> distance(parameterBag_.size());
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::calculateLinearEqualityConstraint(
		Eigen::MatrixXd& Am, Eigen::VectorXd& Bv) {

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
template <size_t STATE_DIM, size_t INPUT_DIM>
bool OCS2Projected<STATE_DIM, INPUT_DIM>::calculateGradient(const size_t& id,
		const Eigen::VectorXd& parameters, Eigen::VectorXd& gradient) {

	// switching time vector
	scalar_array_t switchingTimes(initSwitchingTimes_);
	for (size_t j=0; j<numParameters_; j++)
		switchingTimes[j+1] = parameters(j);

	/*
	 * solve Sequential Riccati Equations with learningRate 0.0,
	 * calculate the nominal co-state,
	 * add the deleted controller parts
	 */
	slqpSolverPtrs_[id]->runExit();

	gslqpSolver_->run(initTime_, initState_, finalTime_, systemStockIndexes_, switchingTimes, slqpSolverPtrs_[id],
			desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
	gslqpSolver_->getCostFuntionDerivative(costFuntionDerivative_);
	gradient = costFuntionDerivative_;

	return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
bool OCS2Projected<STATE_DIM, INPUT_DIM>::calculateCost(const size_t& id,
		const Eigen::VectorXd& parameters, double& cost) {

	// switching time vector
	scalar_array_t switchingTimes(initSwitchingTimes_);
	for (size_t j=0; j<numParameters_; j++)
		switchingTimes[j+1] = parameters(j);

	// initial controller
	controller_array_t controllersStock(numSubsystems_);
	calculateInitialController(initState_, switchingTimes, controllersStock);

	// run SLQP
	try {
		slqpSolverPtrs_[id]->run(initTime_, initState_, finalTime_, systemStockIndexes_, switchingTimes, controllersStock,
				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
	}
	catch (const std::exception& e){
		std::cerr << "\t     exception: " << e.what();
		return false;
	}

	double unconstraintCost, constraintISE;
	slqpSolverPtrs_[id]->getCostFuntion(unconstraintCost, constraintISE);
	cost = unconstraintCost;

	// display
	if (nlpOptions_.displayGradientDescent_)  std::cerr << "\t     constraintISE: " << constraintISE
			<< "\t#Iterations: " << slqpSolverPtrs_[id]->getNumIterations() << std::endl;

	// saving solution in the bag
	saveToBag(id, parameters);

	// status is false if the constraintISE is higher than minAbsConstraint1RMSE_
	bool status = (constraintISE <= options_.minAbsConstraint1ISE_) ? true : false;

	return status;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::saveToBag(size_t id, const Eigen::VectorXd& parameters)  {

	// get the nominal trajectories
	std::vector<scalar_array_t> timeTrajectoriesStock(numSubsystems_);
	state_vector_array2_t   stateTrajectoriesStock(numSubsystems_);
	input_vector_array2_t inputTrajectoriesStock(numSubsystems_);
	slqpSolverPtrs_[id]->getNominalTrajectories(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);

	// get the optimized controller
	controller_array_t controllersStock(numSubsystems_);
	slqpSolverPtrs_[id]->getController(controllersStock);

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

			const double& time = controllersStock[i].time_[k];
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::calculateInitialController(const state_vector_t& initState,
		const scalar_array_t& switchingTimes,
		controller_array_t& controllersStock) {


	// using GLQP for coldStart controller
	controller_array_t coldStartControllersStock(numSubsystems_);

	// calculate the coldStart controllers' cost
	std::vector<scalar_array_t> timeTrajectoriesStock(numSubsystems_);
	state_vector_array2_t stateTrajectoriesStock(numSubsystems_);
	input_vector_array2_t inputTrajectoriesStock(numSubsystems_);
	scalar_t coldStartTotalCost = std::numeric_limits<double>::max();
	try {
		rollout(initTime_, initState_, finalTime_, switchingTimes, coldStartControllersStock,
				timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
		calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock, coldStartTotalCost);
	}
	catch (const std::exception& e){
		std::cerr << "\t     Cold start failed!" << std::endl;
	}


	controller_array_t warmStartControllersStock(numSubsystems_);
	scalar_t warmStartTotalCost = std::numeric_limits<double>::max();
	size_t warmStartIndex;
	if (options_.warmStartGSLQP_==true && parameterBag_.size()>0) {

		// find most similar controller based on the parameter
		const Eigen::VectorXd parameters = Eigen::Map<const Eigen::VectorXd>(&switchingTimes[1], numParameters_);
		warmStartIndex = findNearestController(parameters);

		warmStartControllersStock = controllersStockBag_[warmStartIndex];
		// scaling the controller time to the current switchingTimes
		for (size_t i=0; i<numSubsystems_; i++) {
			double scale = (switchingTimes[i+1]-switchingTimes[i]) / (warmStartControllersStock[i].time_.back()-warmStartControllersStock[i].time_.front());
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
	if (options_.warmStartGSLQP_==true && warmStartTotalCost<=coldStartTotalCost) {
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::rollout(const double& initTime,
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
	size_t initActiveSubsystemIndex = slqp_base_t::findActiveSubsystemIndex(switchingTimes, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = slqp_base_t::findActiveSubsystemIndex(switchingTimes, finalTime);

	double t0 = initTime;
	state_vector_t x0 = initState;
	double tf;
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
		size_t maxNumSteps = options_.maxNumStepsPerSecond_ * std::max(1.0, switchingTimes[i+1]-t0);

		// initialize subsystem i
		subsystemDynamicsPtrStock_[i]->initializeModel(systemStockIndexes_, switchingTimes, x0, i, "GSLPQ");

		// final time
		tf = (i != finalActiveSubsystemIndex) ? switchingTimes[i+1] : finalTime;

		// set controller for subsystem i
		if (controllersStock[i].time_.empty()==false) {
			subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);

		} else {

			if (options_.dispayGSLQP_)  std::cout << "LQP controller is used at period: [" << t0 << ", " << tf << "]" << std::endl;

			controller_t lqpPolicy = lqpControllersStock_[systemStockIndexes_[i]];
			double timeShift = t0-lqpPolicy.time_[0];
			for (size_t k=0; k<lqpPolicy.time_.size(); k++)
				lqpPolicy.time_[k] += timeShift;

			subsystemDynamicsPtrStock_[i]->setController(lqpPolicy);
		}

		// simulate subsystem i
		subsystemSimulatorsStockPtr_[i]->integrate(x0, t0, tf,
				stateTrajectoriesStock[i], timeTrajectoriesStock[i],
				1e-3, options_.AbsTolODE_, options_.RelTolODE_, maxNumSteps);

		if (stateTrajectoriesStock[i].back() != stateTrajectoriesStock[i].back())
			throw std::runtime_error("System became unstable during the SLQP rollout.");

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::calculateCostFunction(
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::getSolution(size_t idStar)  {

	slqpSolverPtrs_[idStar]->getCostFuntion(optimizedTotalCost_, optimizedConstraintISE_);
	slqpSolverPtrs_[idStar]->getSwitchingTimes(optimizedSwitchingTimes_);
	slqpSolverPtrs_[idStar]->getController(optimizedControllersStock_);
	slqpSolverPtrs_[idStar]->getNominalTrajectories(optimizedTimeTrajectoriesStock_, optimizedStateTrajectoriesStock_, optimizedInputTrajectoriesStock_);
	slqpSolverPtrs_[idStar]->getIterationsLog(slqIterationCost_, slqIterationISE1_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::rewindOptimizer(const size_t& firstIndex, bool initRun/*=false*/) {

	// rewind SLQ solvers
	for (size_t i=0; i<slqpSolverPtrs_.size(); i++)
		slqpSolverPtrs_[i]->rewindOptimizer(firstIndex, initRun);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::setUseDisjointRiccati(bool useDisjointRiccati) {
	for (size_t i=0; i<slqpSolverPtrs_.size(); i++)
		slqpSolverPtrs_[i]->setUseDisjointRiccati(useDisjointRiccati);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::setupOptimizer() {

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void OCS2Projected<STATE_DIM, INPUT_DIM>::run(
		const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
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

	for (size_t i=0; i<slqpSolverPtrs_.size(); i++)
		slqpSolverPtrs_[i]->options() = options_;

	// run
	Eigen::VectorXd initParameters = Eigen::Map<Eigen::VectorXd>(initSwitchingTimes_.data()+1, numSubsystems_-1);
	GradientDescent::run(initParameters);

}


}  // end of ocs2 namespace
