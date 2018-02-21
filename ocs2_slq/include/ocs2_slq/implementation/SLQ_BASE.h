/*
 * Implementation of SLQ_BASE.h
 *
 *  Created on: October 7, 2016
 *      Author: farbod
 */


namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::SLQ_BASE(
		const controlled_system_base_t* systemDynamicsPtr,
		const derivatives_base_t* systemDerivativesPtr,
		const constraint_base_t* systemConstraintsPtr,
		const cost_function_base_t* costFunctionPtr,
		const operating_trajectories_base_t* operatingTrajectoriesPtr,
		const SLQ_Settings& settings /*= SLQ_Settings()*/,
		const LOGIC_RULES_T& logicRules /*= LOGIC_RULES_T()*/,
		const cost_function_base_t* heuristicsFunctionPtr /* = nullptr*/)

		: settings_(settings),
		  logicRulesMachine_(logicRules),
		  numPartitionings_(0)
{
	// Dynamics, Constraints, derivatives, and cost
	systemDynamicsPtrStock_.clear();
	systemDynamicsPtrStock_.reserve(settings_.nThreads_);
	systemDerivativesPtrStock_.clear();
	systemDerivativesPtrStock_.reserve(settings_.nThreads_);
	systemConstraintsPtrStock_.clear();
	systemConstraintsPtrStock_.reserve(settings_.nThreads_);
	costFunctionsPtrStock_.clear();
	costFunctionsPtrStock_.reserve(settings_.nThreads_);
	heuristicsFunctionsPtrStock_.clear();
	heuristicsFunctionsPtrStock_.reserve(settings_.nThreads_);
	systemEventHandlersPtrStock_.clear();
	systemEventHandlersPtrStock_.reserve(settings_.nThreads_);
	dynamicsIntegratorsStockPtr_.clear();
	dynamicsIntegratorsStockPtr_.reserve(settings_.nThreads_);

	// initialize all subsystems, etc.
	for (size_t i=0; i<settings_.nThreads_; i++) {

		// initialize dynamics
		systemDynamicsPtrStock_.emplace_back( systemDynamicsPtr->clone() );

		// initialize linearized systems
		systemDerivativesPtrStock_.emplace_back( systemDerivativesPtr->clone() );

		// initialize constraints
		systemConstraintsPtrStock_.emplace_back( systemConstraintsPtr->clone() );

		// initialize cost functions
		costFunctionsPtrStock_.emplace_back( costFunctionPtr->clone() );

		// initialize operating trajectories
		operatingTrajectoriesPtrStock_.emplace_back( operatingTrajectoriesPtr->clone() );

		// initialize heuristics functions
		if (heuristicsFunctionPtr!=nullptr)
			heuristicsFunctionsPtrStock_.emplace_back( heuristicsFunctionPtr->clone() );
		else // use the cost function if no heuristics function is defined
			heuristicsFunctionsPtrStock_.emplace_back( costFunctionPtr->clone() );

		// initialize events
		typedef Eigen::aligned_allocator<event_handler_t> event_handler_alloc_t;
		systemEventHandlersPtrStock_.push_back(std::move( std::allocate_shared<event_handler_t, event_handler_alloc_t>(event_handler_alloc_t()) ));

		// initialize integrators
		typedef ODE45<STATE_DIM> ode_t;
		typedef Eigen::aligned_allocator<ode_t> ode_alloc_t;
		dynamicsIntegratorsStockPtr_.push_back(std::move( std::allocate_shared<ode_t, ode_alloc_t>(
				ode_alloc_t(), systemDynamicsPtrStock_.back(), systemEventHandlersPtrStock_.back()) ));

	} // end of i loop

	// for controller design
	nominalStateFunc_.resize(settings.nThreads_);
	nominalInputFunc_.resize(settings.nThreads_);
	BmFunc_.resize(settings.nThreads_);
	PmFunc_.resize(settings.nThreads_);
	RmInverseFunc_.resize(settings.nThreads_);
	RvFunc_.resize(settings.nThreads_);
	EvProjectedFunc_.resize(settings.nThreads_);
	CmProjectedFunc_.resize(settings.nThreads_);
	DmProjectedFunc_.resize(settings.nThreads_);

	// Riccati Solver
	riccatiEquationsPtrStock_.clear();
	riccatiEquationsPtrStock_.reserve(settings_.nThreads_);
	errorEquationPtrStock_.clear();
	errorEquationPtrStock_.reserve(settings_.nThreads_);
	riccatiIntegratorPtrStock_.clear();
	riccatiIntegratorPtrStock_.reserve(settings_.nThreads_);
	errorIntegratorPtrStock_.clear();
	errorIntegratorPtrStock_.reserve(settings_.nThreads_);


	for (size_t i=0; i<settings_.nThreads_; i++)  {

		typedef Eigen::aligned_allocator<riccati_equations_t> riccati_equations_alloc_t;
		riccatiEquationsPtrStock_.push_back(std::move(
				std::allocate_shared<riccati_equations_t, riccati_equations_alloc_t>(riccati_equations_alloc_t()) ));

		typedef Eigen::aligned_allocator<error_equation_t> error_equation_alloc_t;
		errorEquationPtrStock_.push_back(std::move(
				std::allocate_shared<error_equation_t, error_equation_alloc_t>(error_equation_alloc_t()) ));

		switch(settings_.RiccatiIntegratorType_) {

		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ODE45 : {
			riccatiIntegratorPtrStock_.emplace_back( new ODE45<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back()) );
			errorIntegratorPtrStock_.emplace_back( new ODE45<STATE_DIM>(errorEquationPtrStock_.back()) );
			break;
		}
		/*note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ADAMS_BASHFORTH : {
			throw std::runtime_error("This ADAMS_BASHFORTH is not implemented for Riccati Integrator.");
			break;
		}
		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::BULIRSCH_STOER : {
			riccatiIntegratorPtrStock_.emplace_back( new IntegratorBulirschStoer<riccati_equations_t::S_DIM_>(riccatiEquationsPtrStock_.back()) );
			errorIntegratorPtrStock_.emplace_back( new IntegratorBulirschStoer<STATE_DIM>(errorEquationPtrStock_.back()) );
			break;
		}
		default:
			throw (std::runtime_error("Riccati equation integrator type specified wrongly."));
		}

	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~SLQ_BASE()  {

#ifdef BENCHMARK
	std::cout<<"Avg time for Forward Pass:      " << tAvgFP << " ms " << std::endl;
	std::cout<<"Avg time for Backward Pass:     " << tAvgBP << " ms " << std::endl;
	std::cout<<"Avg time for LQ Approximation:  " << tAvgLQ << " ms " << std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// FIXME: Init time can be bigger than the firt switching time (e.g. MPC seetings)
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_t& controller,
		const scalar_array_t& eventTimes,
		scalar_array_t& timeTrajectory,
		size_array_t& eventsPastTheEndIndeces,
		state_vector_array_t& stateTrajectory,
		input_vector_array_t& inputTrajectory)  {

	// max number of steps for integration
	size_t maxNumSteps = settings_.maxNumStepsPerSecond_ * std::max(1.0, finalTime-initTime);

	// clearing the output trajectories
	timeTrajectory.clear();
	timeTrajectory.reserve(maxNumSteps+1);
	stateTrajectory.clear();
	stateTrajectory.reserve(maxNumSteps+1);
	inputTrajectory.clear();
	inputTrajectory.reserve(maxNumSteps+1);
	eventsPastTheEndIndeces.resize(eventTimes.size());

	if (controller.time_.empty()==false) {
		// set controller
		systemDynamicsPtrStock_[workerIndex]->setController(controller);
		// reset function calls counter
		systemDynamicsPtrStock_[workerIndex]->resetNumFunctionCalls();
		// initialize subsystem
		systemDynamicsPtrStock_[workerIndex]->initializeModel(logicRulesMachine_, partitionIndex, "SLQ");

	} else {
		// initialize operatingTrajectories
		operatingTrajectoriesPtrStock_[workerIndex]->initializeModel(logicRulesMachine_, partitionIndex, "SLQ");
	}

	state_vector_t beginState = initState;
	scalar_t beginTime, endTime;
	size_t k_u = 0;

	for (size_t i=0; i<=eventTimes.size(); i++) {

		beginTime = ( i==0 ? initTime : eventTimes[i-1]) ;
		endTime = ( i==eventTimes.size() ? finalTime : eventTimes[i] );

		// skip if finalTime==eventTimes.back()
		// this is consistent with LogicRulesMachine::findSwitchedSystemsDistribution method
		if (i==eventTimes.size() && eventTimes.empty()==false)
			if (std::abs(finalTime-eventTimes.back()) < OCS2NumericTraits<double>::limit_epsilon())
				continue;

		// simulate subsystem
		if (controller.empty()==false) { // use Base rolloutWorker to use the worker threadId
			// integrate controled system
			dynamicsIntegratorsStockPtr_[workerIndex]->integrate(beginState, beginTime, endTime,
					stateTrajectory, timeTrajectory,
					settings_.minTimeStep_, settings_.absTolODE_, settings_.relTolODE_, maxNumSteps, true);
			// compute control input trajectory and concatinate to inputTrajectory
			for ( ; k_u<timeTrajectory.size(); k_u++) {
				inputTrajectory.emplace_back( systemDynamicsPtrStock_[workerIndex]->computeInput(
						timeTrajectory[k_u], stateTrajectory[k_u]) );
			} // end of k loop

		} else { // if no controller is assigned
			// get operating trajectories
			operatingTrajectoriesPtrStock_[workerIndex]->getSystemOperatingTrajectories(beginState, beginTime, endTime,
					timeTrajectory, stateTrajectory, inputTrajectory,
					true);
		}

		if (i<eventTimes.size()) {
			eventsPastTheEndIndeces[i] = stateTrajectory.size();
			systemDynamicsPtrStock_[workerIndex]->mapState(eventTimes[i], stateTrajectory.back(), beginState);
		}

	}  // end of i loop

}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutTrajectory(
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		input_vector_array2_t& inputTrajectoriesStock,
		size_t threadId /*= 0*/)  {

	size_t numPartitionings = partitioningTimes.size()-1;

	if (controllersStock.size() != numPartitionings)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(numPartitionings);
	eventsPastTheEndIndecesStock.resize(numPartitionings);
	stateTrajectoriesStock.resize(numPartitionings);
	inputTrajectoriesStock.resize(numPartitionings);

	// finding the active subsystem index at initTime
	size_t initActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, finalTime);

	scalar_t t0 = initTime;
	state_vector_t x0 = initState;
	scalar_t tf;
	for (size_t i=0; i<numPartitionings; i++)  {

		// for subsystems before the initial time
		if (i<initActiveSubsystemIndex  ||  i>finalActiveSubsystemIndex) {
			timeTrajectoriesStock[i].clear();
			eventsPastTheEndIndecesStock[i].clear();
			stateTrajectoriesStock[i].clear();
			inputTrajectoriesStock[i].clear();
			continue;
		}

		// final time
		tf = (i != finalActiveSubsystemIndex) ? partitioningTimes[i+1] : finalTime;

		// use Base rolloutWorker to use the worker threadId
		rolloutWorker(threadId, i,
				t0, x0, tf, controllersStock[i], logicRulesMachine_.getSwitchingTimes(i),
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i], stateTrajectoriesStock[i], inputTrajectoriesStock[i]);


		// TODO: delete debug display
//		std::cout << ">> partition: " << i << "\t size: " << timeTrajectoriesStock[i].size() <<std::endl;
//		std::cout << "t0: " << t0 << "\t tf: " << tf <<std::endl;
//		for (size_t j=0; j<timeTrajectoriesStock[i].size(); j++) {
//			std::cout << "state [" << timeTrajectoriesStock[i][j] << "]: " << stateTrajectoriesStock[i][j].head(6).transpose() <<std::endl;
//			std::cout << "input [" << timeTrajectoriesStock[i][j] << "]: " << inputTrajectoriesStock[i][j].head(12).transpose() <<std::endl;
//		}
//		std::cout << std::endl;

		// reset the initial time and state
		t0 = timeTrajectoriesStock[i].back();
		x0 = stateTrajectoriesStock[i].back();

		if (x0 != x0)  throw std::runtime_error("System became unstable during the SLQ rollout.");

	}  // end of i loop
}

/******************************************************************************************************/
// FIXME (speed improvement): inputs are not necessary to be computed for all the time steps
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rolloutFinalState (
		const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const controller_array_t& controllersStock,
		state_vector_t& finalState,
		input_vector_t& finalInput,
		size_t& finalActiveSubsystemIndex,
		size_t threadId /*= 0*/) {

	size_t numPartitionings = partitioningTimes.size()-1;

	if (controllersStock.size() != numPartitionings)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	scalar_array_t 			timeTrajectory;
	size_array_t 			eventsPastTheEndIndeces;
	state_vector_array_t 	stateTrajectory;
	input_vector_t 		inputTrajectory;

	// finding the active subsystem index at initTime and final time
	size_t initActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, initTime);
	finalActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, finalTime);

	scalar_t t0 = initTime, tf;
	state_vector_t x0 = initState;
	for (size_t i=initActiveSubsystemIndex; i<finalActiveSubsystemIndex+1; i++) {

		timeTrajectory.clear();
		stateTrajectory.clear();
		inputTrajectory.clear();

		// final time
		tf = (i != finalActiveSubsystemIndex) ? partitioningTimes[i+1] : finalTime;

		// use Base rolloutWorker to use the worker i
		rolloutWorker(threadId, i,
				t0, x0, tf, controllersStock[i], logicRulesMachine_.getSwitchingTimes(i),
				timeTrajectory, eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

		// reset the initial time and state
		t0 = timeTrajectory.back();
		x0 = stateTrajectory.back();

		if (x0 != x0) throw std::runtime_error("System became unstable during the SLQ rollout.");
	}

	// compute state, control and output
	finalState = stateTrajectory.back();
	finalInput = inputTrajectory.back();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateConstraintsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_array_t& timeTrajectory,
		const size_array_t& eventsPastTheEndIndeces,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory,
		size_array_t& nc1Trajectory,
		constraint1_vector_array_t& EvTrajectory,
		size_array_t& nc2Trajectory,
		constraint2_vector_array_t& HvTrajectory,
		size_array_t& nc2Finals,
		constraint2_vector_array_t& HvFinals) {

	size_t N = timeTrajectory.size();

	// initialize subsystem i constraint
	if (N>0)
		systemConstraintsPtrStock_[workerIndex]->initializeModel(logicRulesMachine_, partitionIndex, "SLQ");

	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1Trajectory.resize(N);
	EvTrajectory.resize(N);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2Trajectory.resize(N);
	HvTrajectory.resize(N);

	nc2Finals.clear();
	nc2Finals.reserve(eventsPastTheEndIndeces.size());
	HvFinals.clear();
	HvFinals.reserve(eventsPastTheEndIndeces.size());

	auto eventsPastTheEndItr = eventsPastTheEndIndeces.begin();

	// compute constraint1 trajectory for subsystem i
	for (size_t k=0; k<N; k++) {

		// set data
		systemConstraintsPtrStock_[workerIndex]->setCurrentStateAndControl(timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);

		// constraint 1 type
		systemConstraintsPtrStock_[workerIndex]->computeConstriant1(nc1Trajectory[k], EvTrajectory[k]);
		if (nc1Trajectory[k] > INPUT_DIM)
			throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");

		// constraint type 2
		systemConstraintsPtrStock_[workerIndex]->computeConstriant2(nc2Trajectory[k], HvTrajectory[k]);
		if (nc2Trajectory[k] > INPUT_DIM)
			throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");

		// switching time state-constrinats
		if (eventsPastTheEndItr!=eventsPastTheEndIndeces.end() && k+1==*eventsPastTheEndItr) {
			size_t nc2Final;
			constraint2_vector_t HvFinal;
			systemConstraintsPtrStock_[workerIndex]->computeFinalConstriant2(nc2Final, HvFinal);
			if (nc2Final > INPUT_DIM)
				throw std::runtime_error("Number of active type-2 constraints at final time should be less-equal to the number of input dimension.");

			nc2Finals.push_back(std::move( nc2Final ));
			HvFinals.push_back(std::move( HvFinal ));
			eventsPastTheEndItr++;
		}

	}  // end of k loop
}


/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutConstraints(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		std::vector<size_array_t>& nc1TrajectoriesStock,
		constraint1_vector_array2_t& EvTrajectoryStock,
		std::vector<size_array_t>& nc2TrajectoriesStock,
		constraint2_vector_array2_t& HvTrajectoryStock,
		std::vector<size_array_t>& nc2FinalStock,
		constraint2_vector_array2_t& HvFinalStock,
		size_t threadId /*= 0*/) {

	// calculate constraint violations
	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1TrajectoriesStock.resize(numPartitionings_);
	EvTrajectoryStock.resize(numPartitionings_);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2TrajectoriesStock.resize(numPartitionings_);
	HvTrajectoryStock.resize(numPartitionings_);
	nc2FinalStock.resize(numPartitionings_);
	HvFinalStock.resize(numPartitionings_);

	for (size_t i=0; i<numPartitionings_; i++) {

		calculateConstraintsWorker(threadId, i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				nc1TrajectoriesStock[i], EvTrajectoryStock[i],
				nc2TrajectoriesStock[i], HvTrajectoryStock[i],
				nc2FinalStock[i], HvFinalStock[i]);
	}  // end of i loop
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
typename SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_t
SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateConstraintISE(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoriesStock,
		scalar_t& constraintISE)  {

	constraintISE = 0.0;
	double maxConstraintNorm = 0.0;

	scalar_t currentSquaredNormError;
	scalar_t nextSquaredNormError;

	for (size_t i=0; i<numPartitionings_; i++)  {

		currentSquaredNormError = 0.0;
		nextSquaredNormError = 0.0;

		for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++)  {

			if (k==0) {
				const size_t& nc1 = nc1TrajectoriesStock[i][0];
				if (nc1>0)
					currentSquaredNormError = EvTrajectoriesStock[i][0].head(nc1).squaredNorm();
				else
					currentSquaredNormError = 0.0;
			} else
				currentSquaredNormError = nextSquaredNormError;

			maxConstraintNorm = ((maxConstraintNorm<currentSquaredNormError)? currentSquaredNormError: maxConstraintNorm);

			const size_t& nc1 = nc1TrajectoriesStock[i][k+1];
			if (nc1>0)
				nextSquaredNormError = EvTrajectoriesStock[i][k+1].head(nc1).squaredNorm();
			else
				nextSquaredNormError = 0.0;

			constraintISE += 0.5 * (currentSquaredNormError+nextSquaredNormError) * (timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);

		}  // end of k loop
	}  // end of i loop

	return sqrt(maxConstraintNorm);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCostWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_array_t& timeTrajectory,
		const size_array_t& eventsPastTheEndIndeces,
		const state_vector_array_t& stateTrajectory,
		const input_vector_array_t& inputTrajectory,
		scalar_t& totalCost)  {

	totalCost = 0.0;

	// initialize subsystem i cost
	costFunctionsPtrStock_[workerIndex]->initializeModel(logicRulesMachine_, partitionIndex, "SLQ");
	// set desired trajectories
	costFunctionsPtrStock_[workerIndex]->setCostNominalTrajectoriesPtr(desiredTimeTrajectoryPtrStock_[partitionIndex],
			desiredStateTrajectoryPtrStock_[partitionIndex], desiredInputTrajectoryPtrStock_[partitionIndex]);

	auto eventsPastTheEndItr = eventsPastTheEndIndeces.begin();

	// integrates the intermediate cost using the trapezoidal approximation method
	scalar_t currentIntermediateCost;
	scalar_t nextIntermediateCost;
	for (size_t k=0; k+1<timeTrajectory.size(); k++) {

		if (k==0) {
			costFunctionsPtrStock_[workerIndex]->setCurrentStateAndControl(timeTrajectory[k], stateTrajectory[k], inputTrajectory[k]);
			costFunctionsPtrStock_[workerIndex]->evaluate(currentIntermediateCost);
		} else
		{
			currentIntermediateCost = nextIntermediateCost;
		}

		// feed next state and control to cost function
		costFunctionsPtrStock_[workerIndex]->setCurrentStateAndControl(timeTrajectory[k+1], stateTrajectory[k+1], inputTrajectory[k+1]);
		// evaluate intermediate cost for next time step
		costFunctionsPtrStock_[workerIndex]->evaluate(nextIntermediateCost);

		totalCost += 0.5*(currentIntermediateCost+nextIntermediateCost)*(timeTrajectory[k+1]-timeTrajectory[k]);

		// terminal cost at switching times
		if (eventsPastTheEndItr!=eventsPastTheEndIndeces.end() && k+1==*eventsPastTheEndItr) {
			scalar_t finalCost;
			costFunctionsPtrStock_[workerIndex]->terminalCost(finalCost);
			totalCost += finalCost;

			eventsPastTheEndItr++;
		}

	}  // end of k loop
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost,
		size_t threadId /*= 0*/)  {

	totalCost = 0.0;

	for (size_t i=0; i<numPartitionings_; i++) {
		scalar_t cost;
		calculateCostWorker(threadId, i,
				timeTrajectoriesStock[i], eventsPastTheEndIndecesStock[i],
				stateTrajectoriesStock[i], inputTrajectoriesStock[i],
				cost);
		totalCost += cost;
	}  // end of i loop

	// calculate the Heuristics function at the final time
	heuristicsFunctionsPtrStock_[threadId]->initializeModel(logicRulesMachine_, finalActivePartition_, "SLQ");
	heuristicsFunctionsPtrStock_[threadId]->setCurrentStateAndControl(
			timeTrajectoriesStock[finalActivePartition_].back(),
			stateTrajectoriesStock[finalActivePartition_].back(),
			inputTrajectoriesStock[finalActivePartition_].back());
	scalar_t sHeuristics;
	heuristicsFunctionsPtrStock_[threadId]->terminalCost(sHeuristics);
	totalCost += sHeuristics;
}

/*****************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCost(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const input_vector_array2_t& inputTrajectoriesStock,
		const scalar_t& constraint2ISE,
		const std::vector<size_array_t>& nc2FinalStock,
		const constraint2_vector_array2_t& HvFinalStock,
		scalar_t& totalCost,
		size_t threadId /*= 0*/) {

	calculateRolloutCost(timeTrajectoriesStock, eventsPastTheEndIndecesStock,
			stateTrajectoriesStock, inputTrajectoriesStock,
			totalCost,
			threadId);

	const scalar_t stateConstraintPenalty = settings_.stateConstraintPenaltyCoeff_ *
			std::pow(settings_.stateConstraintPenaltyBase_, iteration_);

	// ISE of type-2 constraint
	totalCost += 0.5 * stateConstraintPenalty * constraint2ISE;

	// final constraint type 2
	if (settings_.noStateConstraints_==false)
		for (size_t i=0; i<numPartitionings_; i++) {
			for (size_t k=0; k<nc2FinalStock[i].size(); k++) {
				const size_t& nc2Final = nc2FinalStock[i][k];
				totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i][k].head(nc2Final).squaredNorm();
			}  // end of k loop
		}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateOptimalControlProblem()  {

	for (size_t i=0; i<numPartitionings_; i++) {

		// intermediate LQ variables
		size_t N = nominalTimeTrajectoriesStock_[i].size();

		AmTrajectoryStock_[i].resize(N);
		BmTrajectoryStock_[i].resize(N);

		nc1TrajectoriesStock_[i].resize(N);
		EvTrajectoryStock_[i].resize(N);
		CmTrajectoryStock_[i].resize(N);
		DmTrajectoryStock_[i].resize(N);
		nc2TrajectoriesStock_[i].resize(N);
		HvTrajectoryStock_[i].resize(N);
		FmTrajectoryStock_[i].resize(N);

		qTrajectoryStock_[i].resize(N);
		QvTrajectoryStock_[i].resize(N);
		QmTrajectoryStock_[i].resize(N);
		RvTrajectoryStock_[i].resize(N);
		RmTrajectoryStock_[i].resize(N);
		RmInverseTrajectoryStock_[i].resize(N);
		PmTrajectoryStock_[i].resize(N);

		// for constraints
		DmDagerTrajectoryStock_[i].resize(N);
		AmConstrainedTrajectoryStock_[i].resize(N);
		QmConstrainedTrajectoryStock_[i].resize(N);
		QvConstrainedTrajectoryStock_[i].resize(N);
		EvProjectedTrajectoryStock_[i].resize(N);
		CmProjectedTrajectoryStock_[i].resize(N);
		DmProjectedTrajectoryStock_[i].resize(N);
		if (settings_.useRiccatiSolver_==true) {
			RmConstrainedTrajectoryStock_[i].resize(N);
		} else {
			BmConstrainedTrajectoryStock_[i].resize(N);
			PmConstrainedTrajectoryStock_[i].resize(N);
			RvConstrainedTrajectoryStock_[i].resize(N);
		}

		// switching times LQ variables
		size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();

		nc2FinalStock_[i].resize(NE);
		HvFinalStock_[i].resize(NE);
		FmFinalStock_[i].resize(NE);
		qFinalStock_[i].resize(NE);
		QvFinalStock_[i].resize(NE);
		QmFinalStock_[i].resize(NE);


		if (N > 0) {

			for(size_t j=0; j<settings_.nThreads_; j++) {
				// initialize subsystem i dynamics derivatives
				systemDerivativesPtrStock_[j]->initializeModel(logicRulesMachine_, i, "SLQ");
				// initialize subsystem i constraint
				systemConstraintsPtrStock_[j]->initializeModel(logicRulesMachine_, i, "SLQ");
				// initialize subsystem i cost
				costFunctionsPtrStock_[j]->initializeModel(logicRulesMachine_, i, "SLQ");
				// set desired trajectories
				costFunctionsPtrStock_[j]->setCostNominalTrajectoriesPtr(desiredTimeTrajectoryPtrStock_[i],
						desiredStateTrajectoryPtrStock_[i], desiredInputTrajectoryPtrStock_[i]);
			}  // end of j loop

			/*
			 * perform the approximateSubsystemLQ for partition i
			 */
			approximatePartitionLQ(i);

		}

	}  // end of i loop

	// calculate the Heuristics function at the final time
	heuristicsFunctionsPtrStock_[0]->initializeModel(logicRulesMachine_, finalActivePartition_, "SLQ");
	heuristicsFunctionsPtrStock_[0]->setCurrentStateAndControl(
			nominalTimeTrajectoriesStock_[finalActivePartition_].back(),
			nominalStateTrajectoriesStock_[finalActivePartition_].back(),
			nominalInputTrajectoriesStock_[finalActivePartition_].back());
	heuristicsFunctionsPtrStock_[0]->terminalCost(sHeuristics_(0));
	heuristicsFunctionsPtrStock_[0]->terminalCostStateDerivative(SvHeuristics_);
	heuristicsFunctionsPtrStock_[0]->terminalCostStateSecondDerivative(SmHeuristics_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// FIXME: optize the temporal variables: (e.g. Eigen::MatrixXd Cm = CmTrajectoryStock_[i][k].topRows(nc1);)
// FIXME: makePSD is active
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateLQWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	const size_t& i = partitionIndex;
	const size_t& k = timeIndex;

	/*
	 * LINEARIZE SYSTEM DYNAMICS
	 */

	// set data
	systemDerivativesPtrStock_[workerIndex]->setCurrentStateAndControl(
			nominalTimeTrajectoriesStock_[i][k],
			nominalStateTrajectoriesStock_[i][k],
			nominalInputTrajectoriesStock_[i][k]);

	// get results
	systemDerivativesPtrStock_[workerIndex]->getDerivativeState(AmTrajectoryStock_[i][k]);
	systemDerivativesPtrStock_[workerIndex]->getDerivativesControl(BmTrajectoryStock_[i][k]);


	/*
	 * CONSTRAINTS and LINEARIZED CONSTRAINTS
	 */

	// set data
	systemConstraintsPtrStock_[workerIndex]->setCurrentStateAndControl(
			nominalTimeTrajectoriesStock_[i][k],
			nominalStateTrajectoriesStock_[i][k],
			nominalInputTrajectoriesStock_[i][k]);

	// constraint type 1
	systemConstraintsPtrStock_[workerIndex]->computeConstriant1(nc1TrajectoriesStock_[i][k], EvTrajectoryStock_[i][k]);
	if (nc1TrajectoriesStock_[i][k] > INPUT_DIM)
		throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");
	// if constraint type 1 is active
	if (nc1TrajectoriesStock_[i][k] > 0) {
		systemConstraintsPtrStock_[workerIndex]->getConstraint1DerivativesState(CmTrajectoryStock_[i][k]);
		systemConstraintsPtrStock_[workerIndex]->getConstraint1DerivativesControl(DmTrajectoryStock_[i][k]);
	}

	// constraint type 2
	systemConstraintsPtrStock_[workerIndex]->computeConstriant2(nc2TrajectoriesStock_[i][k], HvTrajectoryStock_[i][k]);
	if (nc2TrajectoriesStock_[i][k] > INPUT_DIM)
		throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");
	// if constraint type 2 is active
	if (nc2TrajectoriesStock_[i][k] > 0) {
		systemConstraintsPtrStock_[workerIndex]->getConstraint2DerivativesState(FmTrajectoryStock_[i][k]);
	}


	/*
	 * QUADRATIC APPROXIMATION TO THE COST FUNCTION
	 */

	// set data
	costFunctionsPtrStock_[workerIndex]->setCurrentStateAndControl(
			nominalTimeTrajectoriesStock_[i][k],
			nominalStateTrajectoriesStock_[i][k],
			nominalInputTrajectoriesStock_[i][k]);

	// get results
	costFunctionsPtrStock_[workerIndex]->evaluate(qTrajectoryStock_[i][k](0));
	costFunctionsPtrStock_[workerIndex]->stateDerivative(QvTrajectoryStock_[i][k]);
	costFunctionsPtrStock_[workerIndex]->stateSecondDerivative(QmTrajectoryStock_[i][k]);
	costFunctionsPtrStock_[workerIndex]->controlDerivative(RvTrajectoryStock_[i][k]);
	costFunctionsPtrStock_[workerIndex]->controlSecondDerivative(RmTrajectoryStock_[i][k]);
	RmInverseTrajectoryStock_[i][k] = RmTrajectoryStock_[i][k].inverse();
	costFunctionsPtrStock_[workerIndex]->stateControlDerivative(PmTrajectoryStock_[i][k]);

	/*
	 * Modify the unconstrained LQ coefficients to constrained ones
	 */

	// constraint type 2 coefficients
	const scalar_t stateConstraintPenalty = settings_.stateConstraintPenaltyCoeff_ *
			pow(settings_.stateConstraintPenaltyBase_, iteration_);
	const size_t& nc2 = nc2TrajectoriesStock_[i][k];
	if (nc2 > 0) {
		qTrajectoryStock_[i][k]  += 0.5 * stateConstraintPenalty * HvTrajectoryStock_[i][k].head(nc2).transpose() * HvTrajectoryStock_[i][k].head(nc2);
		QvTrajectoryStock_[i][k] += stateConstraintPenalty * FmTrajectoryStock_[i][k].topRows(nc2).transpose() * HvTrajectoryStock_[i][k].head(nc2);
		QmTrajectoryStock_[i][k] += stateConstraintPenalty * FmTrajectoryStock_[i][k].topRows(nc2).transpose() * FmTrajectoryStock_[i][k].topRows(nc2);
	}

	// constraint type 1 coefficients
	const size_t& nc1 = nc1TrajectoriesStock_[i][k];
	if (nc1 == 0) {
		DmDagerTrajectoryStock_[i][k].setZero();
		EvProjectedTrajectoryStock_[i][k].setZero();
		CmProjectedTrajectoryStock_[i][k].setZero();
		DmProjectedTrajectoryStock_[i][k].setZero();

		AmConstrainedTrajectoryStock_[i][k] = AmTrajectoryStock_[i][k];
		QmConstrainedTrajectoryStock_[i][k] = QmTrajectoryStock_[i][k];
		QvConstrainedTrajectoryStock_[i][k] = QvTrajectoryStock_[i][k];
		if (settings_.useRiccatiSolver_==true) {
			RmConstrainedTrajectoryStock_[i][k] = RmTrajectoryStock_[i][k];
		} else {
			BmConstrainedTrajectoryStock_[i][k] = BmTrajectoryStock_[i][k];
			PmConstrainedTrajectoryStock_[i][k] = PmTrajectoryStock_[i][k];
			RvConstrainedTrajectoryStock_[i][k] = RvTrajectoryStock_[i][k];
		}

	} else {
		Eigen::MatrixXd Cm = CmTrajectoryStock_[i][k].topRows(nc1);
		Eigen::MatrixXd Dm = DmTrajectoryStock_[i][k].topRows(nc1);
		Eigen::MatrixXd Ev = EvTrajectoryStock_[i][k].head(nc1);

		Eigen::MatrixXd RmProjected = ( Dm*RmInverseTrajectoryStock_[i][k]*Dm.transpose() ).inverse();
		Eigen::MatrixXd DmDager = RmInverseTrajectoryStock_[i][k] * Dm.transpose() * RmProjected;

		DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
		EvProjectedTrajectoryStock_[i][k] = DmDager * Ev;
		CmProjectedTrajectoryStock_[i][k] = DmDager * Cm;
		DmProjectedTrajectoryStock_[i][k] = DmDager * Dm;

		control_matrix_t DmNullSpaceProjection = control_matrix_t::Identity() - DmProjectedTrajectoryStock_[i][k];
		state_matrix_t   PmTransDmDagerCm = PmTrajectoryStock_[i][k].transpose()*CmProjectedTrajectoryStock_[i][k];

		AmConstrainedTrajectoryStock_[i][k] = AmTrajectoryStock_[i][k] - BmTrajectoryStock_[i][k]*CmProjectedTrajectoryStock_[i][k];
		QmConstrainedTrajectoryStock_[i][k] = QmTrajectoryStock_[i][k] + Cm.transpose()*RmProjected*Cm - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
		QvConstrainedTrajectoryStock_[i][k] = QvTrajectoryStock_[i][k] - CmProjectedTrajectoryStock_[i][k].transpose()*RvTrajectoryStock_[i][k];
		if (settings_.useRiccatiSolver_==true) {
			RmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * RmTrajectoryStock_[i][k] * DmNullSpaceProjection;
		} else {
			BmConstrainedTrajectoryStock_[i][k] = BmTrajectoryStock_[i][k] * DmNullSpaceProjection;
			PmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * PmTrajectoryStock_[i][k];
			RvConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * RvTrajectoryStock_[i][k];
		}

	}

	// making sure that constrained Qm is PSD
	makePSD(QmConstrainedTrajectoryStock_[i][k]);

	// if a switch took place calculate switch related variables
	size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();
	for (size_t ke=0; ke<NE; ke++)  {
		if (nominalEventsPastTheEndIndecesStock_[i][ke] == k+1)  {

			/*
			 *  Final constraint type 2
			 */
			systemConstraintsPtrStock_[workerIndex]->computeFinalConstriant2(nc2FinalStock_[i][ke], HvFinalStock_[i][ke]);
			if (nc2FinalStock_[i][ke] > INPUT_DIM)
				throw std::runtime_error("Number of active type-2 constraints at final time should be less-equal to the number of input dimension.");
			// if final constraint type 2 is active
			if (nc2FinalStock_[i][ke] > 0)
				systemConstraintsPtrStock_[workerIndex]->getFinalConstraint2DerivativesState(FmFinalStock_[i][ke]);

			/*
			 * Final cost
			 */
			costFunctionsPtrStock_[workerIndex]->terminalCost(qFinalStock_[i][ke](0));
			costFunctionsPtrStock_[workerIndex]->terminalCostStateDerivative(QvFinalStock_[i][ke]);
			costFunctionsPtrStock_[workerIndex]->terminalCostStateSecondDerivative(QmFinalStock_[i][ke]);

			/*
			 * Modify the unconstrained LQ coefficients to constrained ones
			 */
			// final constraint type 2 coefficients
			const size_t& nc2 = nc2FinalStock_[i][ke];
			if (nc2 > 0) {
				qFinalStock_[i][ke]  += 0.5 * stateConstraintPenalty * HvFinalStock_[i][ke].head(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
				QvFinalStock_[i][ke] += stateConstraintPenalty * FmFinalStock_[i][ke].topRows(nc2).transpose() * HvFinalStock_[i][ke].head(nc2);
				QmFinalStock_[i][ke] += stateConstraintPenalty * FmFinalStock_[i][ke].topRows(nc2).transpose() * FmFinalStock_[i][ke].topRows(nc2);
			}

			// making sure that Qm remains PSD
			makePSD(QmFinalStock_[i][ke]);

			break;
		}
	}  // end of ke loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateController() {

	for (size_t i=0; i<numPartitionings_; i++)  {

		if (i<initActivePartition_ || i>finalActivePartition_) {
			nominalControllersStock_[i].clear();
			continue;
		}

		size_t N = SsTimeTrajectoryStock_[i].size();

		nominalControllersStock_[i].time_ = SsTimeTrajectoryStock_[i];
		nominalControllersStock_[i].k_.resize(N);
		nominalControllersStock_[i].uff_.resize(N);
		nominalControllersStock_[i].deltaUff_.resize(N);

		// if the partition is not active
		if (N==0)  continue;

		// initialize interpolators
		for (size_t j = 0; j<settings_.nThreads_; j++) {

			// functions for controller
			nominalStateFunc_[j].reset();
			nominalStateFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			nominalStateFunc_[j].setData( &(nominalStateTrajectoriesStock_[i]) );

			nominalInputFunc_[j].reset();
			nominalInputFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			nominalInputFunc_[j].setData( &(nominalInputTrajectoriesStock_[i]) );

			BmFunc_[j].reset();
			BmFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			BmFunc_[j].setData( &(BmTrajectoryStock_[i]) );

			PmFunc_[j].reset();
			PmFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			PmFunc_[j].setData( &(PmTrajectoryStock_[i]) );

			RmInverseFunc_[j].reset();
			RmInverseFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			RmInverseFunc_[j].setData( &(RmInverseTrajectoryStock_[i]) );

			RvFunc_[j].reset();
			RvFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			RvFunc_[j].setData( &(RvTrajectoryStock_[i]) );

			EvProjectedFunc_[j].reset();
			EvProjectedFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			EvProjectedFunc_[j].setData( &(EvProjectedTrajectoryStock_[i]) );

			CmProjectedFunc_[j].reset();
			CmProjectedFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			CmProjectedFunc_[j].setData( &(CmProjectedTrajectoryStock_[i]) );

			DmProjectedFunc_[j].reset();
			DmProjectedFunc_[j].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
			DmProjectedFunc_[j].setData( &(DmProjectedTrajectoryStock_[i]) );

		}  // end of j loop

		// current partition update
		constraintStepSize_ = updateFeedForwardPoliciesStock_[i] ? settings_.constraintStepSize_ : 0.0;

		/*
		 * perform the calculatePartitionController for partition i
		 */
		calculatePartitionController(i);

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerWorker (
		size_t workerIndex,
		const size_t& partitionIndex,
		const size_t& timeIndex)  {

	const size_t& i = partitionIndex;
	const size_t& k = timeIndex;
	const scalar_t& time = SsTimeTrajectoryStock_[i][k];

	// local variables
	state_vector_t nominalState;
	input_vector_t nominalInput;
	control_gain_matrix_t Bm;
	control_feedback_t Pm;
	input_vector_t Rv;
	control_matrix_t RmInverse;
	input_vector_t EvProjected;
	control_feedback_t CmProjected;
	control_matrix_t DmProjected;
	control_constraint1_matrix_t DmDager;
	control_matrix_t Rm;

	// interpolate
	nominalStateFunc_[workerIndex].interpolate(time, nominalState);
	size_t greatestLessTimeStampIndex = nominalStateFunc_[workerIndex].getGreatestLessTimeStampIndex();
	nominalInputFunc_[workerIndex].interpolate(time, nominalInput, greatestLessTimeStampIndex);

	BmFunc_[workerIndex].interpolate(time, Bm, greatestLessTimeStampIndex);
	PmFunc_[workerIndex].interpolate(time, Pm, greatestLessTimeStampIndex);
	RvFunc_[workerIndex].interpolate(time, Rv, greatestLessTimeStampIndex);
	RmInverseFunc_[workerIndex].interpolate(time, RmInverse, greatestLessTimeStampIndex);
	EvProjectedFunc_[workerIndex].interpolate(time, EvProjected, greatestLessTimeStampIndex);
	CmProjectedFunc_[workerIndex].interpolate(time, CmProjected, greatestLessTimeStampIndex);
	DmProjectedFunc_[workerIndex].interpolate(time, DmProjected, greatestLessTimeStampIndex);

	control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*SmTrajectoryStock_[i][k]);
	input_vector_t     Lv  = RmInverse * (Rv + Bm.transpose()*SvTrajectoryStock_[i][k]);
	input_vector_t     Lve = RmInverse * (Bm.transpose()*SveTrajectoryStock_[i][k]);

	control_matrix_t DmNullProjection = control_matrix_t::Identity()-DmProjected;
	nominalControllersStock_[i].k_[k]   = -DmNullProjection*Lm - CmProjected;
	nominalControllersStock_[i].uff_[k] = nominalInput - nominalControllersStock_[i].k_[k]*nominalState
			- constraintStepSize_ * (DmNullProjection*Lve + EvProjected);
	nominalControllersStock_[i].deltaUff_[k] = -DmNullProjection*Lv;

	// checking the numerical stability of the controller parameters
	if (settings_.checkNumericalStability_==true){
		try {
			if (nominalControllersStock_[i].k_[k] != nominalControllersStock_[i].k_[k])
				throw std::runtime_error("Feedback gains are unstable.");
			if (nominalControllersStock_[i].deltaUff_[k] != nominalControllersStock_[i].deltaUff_[k])
				throw std::runtime_error("feedForwardControl is unstable.");
		}
		catch(const std::exception& error)  {
			std::cerr << "what(): " << error.what() << " at time " << nominalControllersStock_[i].time_[k] << " [sec]." << std::endl;
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::lineSearchWorker(
		size_t workerIndex,
		scalar_t learningRate,
		scalar_t& lsTotalCost,
		scalar_t& lsConstraint1ISE, scalar_t& lsConstraint1MaxNorm,
		scalar_t& lsConstraint2ISE, scalar_t& lsConstraint2MaxNorm,
		controller_array_t& lsControllersStock,
		std::vector<scalar_array_t>& lsTimeTrajectoriesStock,
		std::vector<size_array_t>& lsEventsPastTheEndIndecesStock,
		state_vector_array2_t& lsStateTrajectoriesStock,
		input_vector_array2_t& lsInputTrajectoriesStock)  {

	// modifying uff by local increments
	for (size_t i=0; i<numPartitionings_; i++)
		for (size_t k=0; k<lsControllersStock[i].time_.size(); k++)
			lsControllersStock[i].uff_[k] += learningRate * lsControllersStock[i].deltaUff_[k];

	try {
		// perform a rollout
		rolloutTrajectory(initTime_, initState_, finalTime_,
				partitioningTimes_, lsControllersStock,
				lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				workerIndex);

		// calculate rollout constraints
		std::vector<size_array_t>   lsNc1TrajectoriesStock(numPartitionings_);
		constraint1_vector_array2_t lsEvTrajectoryStock(numPartitionings_);
		std::vector<size_array_t>   lsNc2TrajectoriesStock(numPartitionings_);
		constraint2_vector_array2_t lsHvTrajectoryStock(numPartitionings_);
		std::vector<size_array_t>	lsNc2FinalStock(numPartitionings_);
		constraint2_vector_array2_t	lsHvFinalStock(numPartitionings_);

		if (lsComputeISEs_==true) {
			// calculate rollout constraints
			calculateRolloutConstraints(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
					lsStateTrajectoriesStock, lsInputTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,
					lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock,
					workerIndex);
			// calculate constraint type-1 ISE and maximum norm
			lsConstraint1MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,
					lsConstraint1ISE);
			// calculates type-2 constraint ISE and maximum norm
			lsConstraint2MaxNorm = calculateConstraintISE(lsTimeTrajectoriesStock,
					lsNc2TrajectoriesStock, lsHvTrajectoryStock,
					lsConstraint2ISE);
		} else {
			lsConstraint1ISE = lsConstraint1MaxNorm = 0.0;
			lsConstraint2ISE = lsConstraint2MaxNorm = 0.0;
		}

		// calculate rollout cost
		calculateRolloutCost(lsTimeTrajectoriesStock, lsEventsPastTheEndIndecesStock,
				lsStateTrajectoriesStock, lsInputTrajectoriesStock,
				lsConstraint2ISE, lsNc2FinalStock, lsHvFinalStock,
				lsTotalCost,
				workerIndex);

		// display
		if (settings_.displayInfo_) {
			printString("\t [Thread" + std::to_string(workerIndex) + "] - learningRate " + std::to_string(learningRate)
					+ " \t cost: " + std::to_string(lsTotalCost) + " \t constraint ISE: " + std::to_string(lsConstraint1ISE));

			std::string finalConstraintDisplay = "\t final constraint type-2:   ";
			for(size_t i=0; i<numPartitionings_; i++) {
				finalConstraintDisplay += "[" + std::to_string(i) + "]: ";
				for (size_t j=0; j<lsNc2FinalStock[i].size(); j++)
					for (size_t m=0; m<lsNc2FinalStock[i][j]; m++)
						finalConstraintDisplay += std::to_string(lsHvFinalStock[i][j](m)) + ", ";
				finalConstraintDisplay += "  ";
			} // end of i loop
			printString(finalConstraintDisplay);
		}

	} catch(const std::exception& error) {
		lsTotalCost  = std::numeric_limits<scalar_t>::max();
		if(settings_.displayInfo_)
			printString("rollout with learningRate " + std::to_string(learningRate) + "\t" +  error.what());
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	// set data for Riccati equations
	riccatiEquationsPtrStock_[workerIndex]->reset();
	riccatiEquationsPtrStock_[workerIndex]->setData(partitioningTimes_[partitionIndex], partitioningTimes_[partitionIndex+1],
			&nominalTimeTrajectoriesStock_[partitionIndex],
			&AmConstrainedTrajectoryStock_[partitionIndex], &BmTrajectoryStock_[partitionIndex],
			&qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex], &QmConstrainedTrajectoryStock_[partitionIndex],
			&RvTrajectoryStock_[partitionIndex], &RmInverseTrajectoryStock_[partitionIndex], &RmConstrainedTrajectoryStock_[partitionIndex],
			&PmTrajectoryStock_[partitionIndex],
			&nominalEventsPastTheEndIndecesStock_[partitionIndex],
			&qFinalStock_[partitionIndex], &QvFinalStock_[partitionIndex], &QmFinalStock_[partitionIndex]);

	// Normalized start and final time for reccati equation
	scalar_t stratNormalizedTime = 0.0;
	scalar_t finalNormalizedTime = 1.0;
	if (partitionIndex==initActivePartition_) {
		finalNormalizedTime = (initTime_-partitioningTimes_[partitionIndex+1])  / (partitioningTimes_[partitionIndex]-partitioningTimes_[partitionIndex+1]);
	} else if (partitionIndex==finalActivePartition_) {
		stratNormalizedTime = (finalTime_-partitioningTimes_[partitionIndex+1]) / (partitioningTimes_[partitionIndex]-partitioningTimes_[partitionIndex+1]);
	}

	// Normalized event times
	size_t NE = logicRulesMachine_.getSwitchingTimes(partitionIndex).size();
	scalar_array_t SsNormalizedEventTimes(NE);
	for (size_t k=0; k<NE; k++) {
		const scalar_t& eventTime = logicRulesMachine_.getSwitchingTimes(partitionIndex).at(k);
		SsNormalizedEventTimes[NE-1-k] = (eventTime-partitioningTimes_[partitionIndex+1])  / (partitioningTimes_[partitionIndex]-partitioningTimes_[partitionIndex+1]);
	}

	// max number of steps of integration
	riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
	size_t maxNumSteps = settings_.maxNumStepsPerSecond_ * std::max(1.0, finalNormalizedTime-stratNormalizedTime);

	// final value for the last Riccati equations plus final cost
	typename riccati_equations_t::s_vector_t allSsFinal;
	riccati_equations_t::convert2Vector(SmFinal, SvFinal, sFinal, allSsFinal);

	// clear output containers
	SsNormalizedTimeTrajectoryStock_[partitionIndex].clear();
	SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
	typename riccati_equations_t::s_vector_array_t allSsTrajectory;

	scalar_t beginTime, endTime;
	for (size_t i=0; i<=NE; i++) {

		beginTime = ( i==0 ? stratNormalizedTime : SsNormalizedEventTimes[i-1]) ;
		endTime = ( i==NE ? finalNormalizedTime : SsNormalizedEventTimes[i] );

		// skip if finalTime==eventTimes.back().
		// This is consistent with LogicRulesMachine::findSwitchedSystemsDistribution method
		if (i==0 && NE>0 && std::abs(stratNormalizedTime-SsNormalizedEventTimes.front())<OCS2NumericTraits<double>::limit_epsilon()) {
			SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][0] = 0;
			typename riccati_equations_t::s_vector_t allSsFinalTemp = allSsFinal;
			riccatiEquationsPtrStock_[workerIndex]->mapState(endTime, allSsFinalTemp, allSsFinal);
			continue;
		}

		// solve Riccati equations
		riccatiIntegratorPtrStock_[workerIndex]->integrate(allSsFinal, beginTime, endTime,
				allSsTrajectory, SsNormalizedTimeTrajectoryStock_[partitionIndex],
				settings_.minTimeStep_, settings_.absTolODE_, settings_.relTolODE_, maxNumSteps, true);

		if (i<NE) {
			SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][i] = allSsTrajectory.size();
			riccatiEquationsPtrStock_[workerIndex]->mapState(endTime, allSsTrajectory.back(), allSsFinal);
		}

	}  // end of i loop


	// denormalizing time and constructing 'Sm', 'Sv', and 's'
	size_t N = SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
	SsTimeTrajectoryStock_[partitionIndex].resize(N);
	SmTrajectoryStock_[partitionIndex].resize(N);
	SvTrajectoryStock_[partitionIndex].resize(N);
	sTrajectoryStock_[partitionIndex].resize(N);
	for (size_t k=0; k<N; k++) {
		riccati_equations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[partitionIndex][k], SvTrajectoryStock_[partitionIndex][k], sTrajectoryStock_[partitionIndex][k]);
//		SsTimeTrajectoryStock_[partitionIndex][k] = partitioningTimes_[partitionIndex+1] - SsNormalizedTimeTrajectoryStock_[partitionIndex][N-1-k];
		SsTimeTrajectoryStock_[partitionIndex][k] = (partitioningTimes_[partitionIndex]-partitioningTimes_[partitionIndex+1])*SsNormalizedTimeTrajectoryStock_[partitionIndex][N-1-k] +
				partitioningTimes_[partitionIndex+1];
	}  // end of k loop

	// TODO: delete debug display
//	std::cout << "\n>>> Subsystem: " << partitionIndex << std::endl << std::endl;
//	for (size_t j=0; j<SsTimeTrajectoryStock_[partitionIndex].size(); j++)
//		std::cout << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][j] << "]:\n" << SmTrajectoryStock_[partitionIndex][j].template topLeftCorner<12,12>() << std::endl;

//	// set the final value for next Riccati equation
//	sFinalStock_[partitionIndex]  = sTrajectoryStock_[partitionIndex].front();
//	SvFinalStock_[partitionIndex] = SvTrajectoryStock_[partitionIndex].front();
//	SmFinalStock_[partitionIndex] = SmTrajectoryStock_[partitionIndex].front();

	// testing the numerical stability of the Riccati equations
	if (settings_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (SmTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sv is unstable.");
				if (sTrajectoryStock_[partitionIndex][k].hasNaN())   throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"<< SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"<< sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveRiccatiEquationsWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const scalar_array_t& nominalTimeTrajectory,
		const size_array_t& nominalEventsPastTheEndIndeces,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

	// set data for Riccati equations
	riccatiEquationsPtrStock_[workerIndex]->reset();
	riccatiEquationsPtrStock_[workerIndex]->setData(partitioningTimes_[partitionIndex], partitioningTimes_[partitionIndex+1],
			&nominalTimeTrajectory,
			&AmConstrainedTrajectoryStock_[partitionIndex], &BmTrajectoryStock_[partitionIndex],
			&qTrajectoryStock_[partitionIndex], &QvConstrainedTrajectoryStock_[partitionIndex], &QmConstrainedTrajectoryStock_[partitionIndex],
			&RvTrajectoryStock_[partitionIndex], &RmInverseTrajectoryStock_[partitionIndex], &RmConstrainedTrajectoryStock_[partitionIndex],
			&PmTrajectoryStock_[partitionIndex],
			&nominalEventsPastTheEndIndecesStock_[partitionIndex],
			&qFinalStock_[partitionIndex], &QvFinalStock_[partitionIndex], &QmFinalStock_[partitionIndex]);

	// final value for the last Riccati equations plus final cost
	typename riccati_equations_t::s_vector_t allSsFinal;
	riccati_equations_t::convert2Vector(SmFinal+QmFinalStock_[partitionIndex], SvFinal+QvFinalStock_[partitionIndex], sFinal+qFinalStock_[partitionIndex], allSsFinal);

	size_t N = nominalTimeTrajectory.size();
	SsNormalizedTimeTrajectoryStock_[partitionIndex].resize(N);
	for (size_t k=0; k<N; k++)
		SsNormalizedTimeTrajectoryStock_[partitionIndex][N-1-k] = (nominalTimeTrajectory[k] - partitioningTimes_[partitionIndex+1])
				/ (partitioningTimes_[partitionIndex]-partitioningTimes_[partitionIndex+1]);

	size_t NE = nominalEventsPastTheEndIndeces.size();
	SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].resize(NE);
	for (size_t k=0; k<NE; k++)
		SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex][NE-1-k] = N - nominalEventsPastTheEndIndeces[k];

	// max number of steps of integration
	riccatiEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
	size_t maxNumSteps = settings_.maxNumStepsPerSecond_ *
			std::max(1.0, *SsNormalizedTimeTrajectoryStock_[partitionIndex].back()-*SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

	typename riccati_equations_t::s_vector_array_t allSsTrajectory;

	typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
	for (size_t i=0; i<=NE; i++) {

		beginTimeItr = ( i==0  ? SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() :
				SsNormalizedTimeTrajectoryStock_[partitionIndex].begin()+SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].at(i-1) );
		endTimeItr   = ( i==NE ? SsNormalizedTimeTrajectoryStock_[partitionIndex].end()   :
				SsNormalizedTimeTrajectoryStock_[partitionIndex].begin()+SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].at(i) );

		// skip if finalTime==eventTimes.back().
		// This is consistent with LogicRulesMachine::findSwitchedSystemsDistribution method
		if (i==0 && NE>0 && std::abs(*endTimeItr-*beginTimeItr)<OCS2NumericTraits<double>::limit_epsilon()) {
			typename riccati_equations_t::s_vector_t allSsFinalTemp = allSsFinal;
			riccatiEquationsPtrStock_[workerIndex]->mapState(*endTimeItr, allSsFinalTemp, allSsFinal);
			continue;
		}

		// solve Riccati equations
		riccatiIntegratorPtrStock_[workerIndex]->integrate(allSsFinal, beginTimeItr, endTimeItr,
				allSsTrajectory,
				settings_.minTimeStep_, settings_.absTolODE_, settings_.relTolODE_, maxNumSteps, true);

		if (i<NE) {
			riccatiEquationsPtrStock_[workerIndex]->mapState(*endTimeItr, allSsTrajectory.back(), allSsFinal);
		}

	}  // end of i loop


	// denormalizing time and constructing 'Sm', 'Sv', and 's'
	SsTimeTrajectoryStock_[partitionIndex] = nominalTimeTrajectory;
	SmTrajectoryStock_[partitionIndex].resize(N);
	SvTrajectoryStock_[partitionIndex].resize(N);
	sTrajectoryStock_[partitionIndex].resize(N);
	for (size_t k=0; k<N; k++) {
		riccati_equations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[partitionIndex][k], SvTrajectoryStock_[partitionIndex][k], sTrajectoryStock_[partitionIndex][k]);
	}  // end of k loop

//	// set the final value for next Riccati equation
//	sFinalStock_[partitionIndex]  = sTrajectoryStock_[partitionIndex].front();
//	SvFinalStock_[partitionIndex] = SvTrajectoryStock_[partitionIndex].front();
//	SmFinalStock_[partitionIndex] = SmTrajectoryStock_[partitionIndex].front();

	// testing the numerical stability of the Riccati equations
	if (settings_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (SmTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sv is unstable.");
				if (sTrajectoryStock_[partitionIndex][k].hasNaN())   throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"<< SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"<< sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveErrorRiccatiEquationWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_vector_t& SveFinal)  {
	/*
	 * Type_1 constraints error correction compensation
	 */

	size_t N  = SsNormalizedTimeTrajectoryStock_[partitionIndex].size();
	size_t NE = nominalEventsPastTheEndIndecesStock_[partitionIndex].size();

	// Skip calculation of the error correction term (Sve) if the constrained simulation is used for forward simulation
	if (settings_.simulationIsConstrained_) {
		SveTrajectoryStock_[partitionIndex].resize(N);
		for (size_t k=0; k<N; k++)
			SveTrajectoryStock_[partitionIndex][k].setZero();
		return;
	}

	/*
	 * Calculating the coefficients of the error equation
	 */
	state_vector_array_t GvTrajectory(nominalTimeTrajectoriesStock_[partitionIndex].size());
	state_matrix_array_t GmTrajectory(nominalTimeTrajectoriesStock_[partitionIndex].size());

	LinearInterpolation<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > SmFunc;
	SmFunc.setTimeStamp( &(SsTimeTrajectoryStock_[partitionIndex]) );
	SmFunc.setData( &(SmTrajectoryStock_[partitionIndex]) );

	for (int k=nominalTimeTrajectoriesStock_[partitionIndex].size()-1; k>=0; k--) {

		state_matrix_t Sm;
		SmFunc.interpolate(nominalTimeTrajectoriesStock_[partitionIndex][k], Sm);

		control_feedback_t Lm = RmInverseTrajectoryStock_[partitionIndex][k]*(PmTrajectoryStock_[partitionIndex][k]+BmTrajectoryStock_[partitionIndex][k].transpose()*Sm);

		GmTrajectory[k] = AmConstrainedTrajectoryStock_[partitionIndex][k] -
				BmTrajectoryStock_[partitionIndex][k]*RmInverseTrajectoryStock_[partitionIndex][k]*RmConstrainedTrajectoryStock_[partitionIndex][k]*Lm;

		GvTrajectory[k] = (CmProjectedTrajectoryStock_[partitionIndex][k]-Lm).transpose()*
				RmTrajectoryStock_[partitionIndex][k]*EvProjectedTrajectoryStock_[partitionIndex][k];

	}  // end of k loop

	// set data for error equations
	errorEquationPtrStock_[workerIndex]->reset();
	errorEquationPtrStock_[workerIndex]->setData(partitioningTimes_[partitionIndex], partitioningTimes_[partitionIndex+1],
			&nominalTimeTrajectoriesStock_[partitionIndex], &GvTrajectory, &GmTrajectory);

	// max number of steps of integration
	errorEquationPtrStock_[workerIndex]->resetNumFunctionCalls();
	size_t maxNumSteps = settings_.maxNumStepsPerSecond_
			* std::max(1.0, SsNormalizedTimeTrajectoryStock_[partitionIndex].back()-SsNormalizedTimeTrajectoryStock_[partitionIndex].front());

	state_vector_array_t SveTrajectory;

	state_vector_t SveFinalInternal = SveFinal;

	// integrating the Error Riccati equation
	typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
	for (size_t i=0; i<=NE; i++) {

		beginTimeItr = ( i==0  ? SsNormalizedTimeTrajectoryStock_[partitionIndex].begin() :
				SsNormalizedTimeTrajectoryStock_[partitionIndex].begin()+SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].at(i-1) );
		endTimeItr   = ( i==NE ? SsNormalizedTimeTrajectoryStock_[partitionIndex].end() :
				SsNormalizedTimeTrajectoryStock_[partitionIndex].begin()+SsNormalizedEventsPastTheEndIndecesStock_[partitionIndex].at(i) )	;

		// skip if finalTime==eventTimes.back().
		// This is consistent with LogicRulesMachine::findSwitchedSystemsDistribution method
		if (i==0 && NE>0 && std::abs(*endTimeItr-*beginTimeItr)<OCS2NumericTraits<double>::limit_epsilon()) {
			errorEquationPtrStock_[workerIndex]->mapState(*endTimeItr, SveFinal, SveFinalInternal);
			continue;
		}

		// solve Riccati equations
		errorIntegratorPtrStock_[workerIndex]->integrate(SveFinalInternal, beginTimeItr, endTimeItr,
				SveTrajectory,
				settings_.minTimeStep_, settings_.absTolODE_, settings_.relTolODE_, maxNumSteps, true);

		if (i<NE) {
			errorEquationPtrStock_[workerIndex]->mapState(*endTimeItr, SveTrajectory.back(), SveFinalInternal);
		}

	}  // end of i loop


//	// set the final value for next Riccati equation
//	SveFinalStock_[partitionIndex] = SveTrajectory.back();

	// constructing Sve
	SveTrajectoryStock_[partitionIndex].resize(N);
	std::reverse_copy(SveTrajectory.begin(), SveTrajectory.end(), SveTrajectoryStock_[partitionIndex].begin());

	// Testing the numerical stability
	if (settings_.checkNumericalStability_==true)
		for (size_t k=0; k<N; k++) {

			// testing the numerical stability of the Riccati error equation
			try {
				if (SveTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sve is unstable");
			}
			catch(const std::exception& error) 	{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<N; kp++){
					std::cerr << "Sve[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				for(int kp = 0; kp+1<nominalTimeTrajectoriesStock_[partitionIndex].size(); kp++){
					std::cerr << "Gm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< GmTrajectory[kp].transpose().norm() << std::endl;
					std::cerr << "Gv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< GvTrajectory[kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}  // end of k loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
template<size_t DIM1, size_t DIM2>
Eigen::Matrix<double, DIM1, DIM2> SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveLTIMatrix(
		const Eigen::Matrix<double, DIM1, DIM1>& A,
		const Eigen::Matrix<double, DIM1, DIM2>& x0,
		const double& deltaTime) {

	// dx = A x
	// deltaTime = t-t0

//	Eigen::HessenbergDecomposition<Eigen::MatrixXd> hessA(DIM1);
//	hessA.compute(A);
//
//	Eigen::Matrix<double, DIM1, DIM2> xf = hessA.matrixQ()
//			* (hessA.matrixH()*deltaTime + Eigen::MatrixXd::Identity(DIM1, DIM1)*1e-3*deltaTime).exp()
//			* hessA.matrixQ().transpose() * x0;

	Eigen::Matrix<double, DIM1, DIM2> xf = (A*deltaTime).exp() * x0;
	return xf;

//	Eigen::EigenSolver<Eigen::MatrixXd> esA(DIM1);
//	esA.compute(A*deltaTime, /* computeEigenvectors = */ true);
//	Eigen::VectorXcd expLamda = esA.eigenvalues().array().exp();
//	Eigen::MatrixXcd V = esA.eigenvectors();
//
//	Eigen::Matrix<double, DIM1, DIM2> xf = (V * expLamda.asDiagonal() * V.inverse()).real() * x0;
//	return xf;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
template<int DIM1>
Eigen::Matrix<double, DIM1, 1> SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveLTI(
		const Eigen::Matrix<double, DIM1, DIM1>& Gm,
		const Eigen::Matrix<double, DIM1, 1>& Gv,
		const Eigen::Matrix<double, DIM1, 1>& x0,
		const double& deltaTime) {

	// dx = A x + B u
	// set data for error equations
	typedef LTI_Equations<DIM1> LTI_Equation_t;
	std::shared_ptr<LTI_Equation_t> ltiEquationPtr( new LTI_Equation_t() );
	ODE45<DIM1> firstOrderOde45(ltiEquationPtr);
	ltiEquationPtr->setData(Gm, Gv);

	scalar_array_t timeTrajectory {0.0, deltaTime};
	std::vector<Eigen::Matrix<double, DIM1, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, 1>> > stateTrajectory;
	firstOrderOde45.integrate(x0, timeTrajectory.begin(), timeTrajectory.end(),
			stateTrajectory,
			settings_.minTimeStep_, settings_.absTolODE_, settings_.relTolODE_);

	Eigen::Matrix<double, DIM1, 1> xf = stateTrajectory.back();

	return xf;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::fullRiccatiBackwardSweepWorker(
		size_t workerIndex,
		const size_t& partitionIndex,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
		const state_vector_t& SveFinal, const eigen_scalar_t& sFinal,
		const scalar_t& constraintStepSize)  {

	size_t N = nominalTimeTrajectoriesStock_[partitionIndex].size();

	state_matrix_t _Gm;
	state_vector_t _Gv, _Gve;
	control_feedback_t _Lm, _LmConstrained;
	input_vector_t _LvConstrained, _LveConstrained;
	Eigen::Matrix<scalar_t, 2*STATE_DIM, STATE_DIM> _X_H_0, _X_H_1;
	Eigen::Matrix<scalar_t, 2*STATE_DIM, 2*STATE_DIM> _H;
	_X_H_0.template topRows<STATE_DIM>() = state_matrix_t::Identity();

	// Riccati parameters
	SmTrajectoryStock_[partitionIndex].resize(N);
	SvTrajectoryStock_[partitionIndex].resize(N);
	SveTrajectoryStock_[partitionIndex].resize(N);
	sTrajectoryStock_[partitionIndex].resize(N);

	SsTimeTrajectoryStock_[partitionIndex]   = nominalTimeTrajectoriesStock_[partitionIndex];
	SmTrajectoryStock_[partitionIndex][N-1]  = SmFinal;
	SvTrajectoryStock_[partitionIndex][N-1]  = SvFinal;
	SveTrajectoryStock_[partitionIndex][N-1] = SveFinal;
	sTrajectoryStock_[partitionIndex][N-1]   = sFinal;

	// controller parameters
	nominalControllersStock_[partitionIndex].time_ = nominalTimeTrajectoriesStock_[partitionIndex];
	nominalControllersStock_[partitionIndex].k_.resize(N);
	nominalControllersStock_[partitionIndex].uff_.resize(N);
	nominalControllersStock_[partitionIndex].deltaUff_.resize(N);

	_Lm 		   = -RmInverseTrajectoryStock_[partitionIndex][N-1] * (
			PmTrajectoryStock_[partitionIndex][N-1] + BmTrajectoryStock_[partitionIndex][N-1].transpose()*SmTrajectoryStock_[partitionIndex][N-1]);
	_LmConstrained = (control_matrix_t::Identity() - DmProjectedTrajectoryStock_[partitionIndex][N-1]) * _Lm;
	_LvConstrained = -RmInverseTrajectoryStock_[partitionIndex][N-1] *
			(RvConstrainedTrajectoryStock_[partitionIndex][N-1] + BmConstrainedTrajectoryStock_[partitionIndex][N-1].transpose()*SvTrajectoryStock_[partitionIndex][N-1]);
	_LveConstrained = -RmInverseTrajectoryStock_[partitionIndex][N-1] * BmConstrainedTrajectoryStock_[partitionIndex][N-1].transpose() * SveTrajectoryStock_[partitionIndex][N-1];
	nominalControllersStock_[partitionIndex].k_[N-1]   = _LmConstrained - CmProjectedTrajectoryStock_[partitionIndex][N-1];
	nominalControllersStock_[partitionIndex].uff_[N-1] = nominalInputTrajectoriesStock_[partitionIndex][N-1]
			- nominalControllersStock_[partitionIndex].k_[N-1]*nominalStateTrajectoriesStock_[partitionIndex][N-1]
			+ constraintStepSize * (_LveConstrained - EvProjectedTrajectoryStock_[partitionIndex][N-1]);
	nominalControllersStock_[partitionIndex].deltaUff_[N-1] = _LvConstrained;

	int remainingEvents = nominalEventsPastTheEndIndecesStock_[partitionIndex].size();
	bool eventDetected = false;

	for (int k=N-2; k>=0; k--) {

		// check if an event is detected.
		if (remainingEvents>0 && k+1==nominalEventsPastTheEndIndecesStock_[partitionIndex][remainingEvents-1]) {
			eventDetected = true;
			remainingEvents--;
		} else {
			eventDetected = false;
		}

		double deltaT = nominalTimeTrajectoriesStock_[partitionIndex][k] - nominalTimeTrajectoriesStock_[partitionIndex][k+1];

		if (eventDetected==true) {
			makePSD(QmFinalStock_[partitionIndex][remainingEvents]);
			SmTrajectoryStock_[partitionIndex][k]  = SmTrajectoryStock_[partitionIndex][k+1] + QmFinalStock_[partitionIndex][remainingEvents];

		} else {
			makePSD(QmConstrainedTrajectoryStock_[partitionIndex][k]);
			_H.template topLeftCorner<STATE_DIM,STATE_DIM>() = AmConstrainedTrajectoryStock_[partitionIndex][k] -
					BmConstrainedTrajectoryStock_[partitionIndex][k]*RmInverseTrajectoryStock_[partitionIndex][k]*PmConstrainedTrajectoryStock_[partitionIndex][k];
			_H.template topRightCorner<STATE_DIM,STATE_DIM>() = 0.5 * BmConstrainedTrajectoryStock_[partitionIndex][k] *
					RmInverseTrajectoryStock_[partitionIndex][k] * BmConstrainedTrajectoryStock_[partitionIndex][k].transpose();
			_H.template bottomLeftCorner<STATE_DIM,STATE_DIM>() = 2.0 * (QmConstrainedTrajectoryStock_[partitionIndex][k] -
					PmConstrainedTrajectoryStock_[partitionIndex][k].transpose()*RmInverseTrajectoryStock_[partitionIndex][k]*PmConstrainedTrajectoryStock_[partitionIndex][k]);
			_H.template bottomRightCorner<STATE_DIM,STATE_DIM>() = -_H.template topLeftCorner<STATE_DIM,STATE_DIM>().transpose();

			_X_H_0.template bottomRows<STATE_DIM>() = -2.0*SmTrajectoryStock_[partitionIndex][k+1];
			_X_H_1 = solveLTIMatrix<2*STATE_DIM, STATE_DIM>(_H, _X_H_0, deltaT);
			SmTrajectoryStock_[partitionIndex][k] = -0.5 * _X_H_1.template bottomRows<STATE_DIM>() * _X_H_1.template topRows<STATE_DIM>().inverse();
		}

		_Lm = -RmInverseTrajectoryStock_[partitionIndex][k] * (
				PmTrajectoryStock_[partitionIndex][k] + BmTrajectoryStock_[partitionIndex][k].transpose()*SmTrajectoryStock_[partitionIndex][k] );
		_LmConstrained = (control_matrix_t::Identity() - DmProjectedTrajectoryStock_[partitionIndex][k]) * _Lm;

		if (eventDetected==true) {
			SvTrajectoryStock_[partitionIndex][k]  = SvTrajectoryStock_[partitionIndex][k+1] + QvFinalStock_[partitionIndex][remainingEvents];
			SveTrajectoryStock_[partitionIndex][k] = SveTrajectoryStock_[partitionIndex][k+1];
			sTrajectoryStock_[partitionIndex][k]   = sTrajectoryStock_[partitionIndex][k+1] + qFinalStock_[partitionIndex][remainingEvents];

		} else {
			_Gm  = (AmConstrainedTrajectoryStock_[partitionIndex][k] + BmConstrainedTrajectoryStock_[partitionIndex][k]*_LmConstrained).transpose();
			_Gv  = (QvConstrainedTrajectoryStock_[partitionIndex][k] + _LmConstrained.transpose()*RvConstrainedTrajectoryStock_[partitionIndex][k]);
			_Gve = (CmProjectedTrajectoryStock_[partitionIndex][k] + _Lm).transpose() * RmTrajectoryStock_[partitionIndex][k] * EvProjectedTrajectoryStock_[partitionIndex][k];

			SvTrajectoryStock_[partitionIndex][k]  = solveLTI<STATE_DIM>(_Gm, _Gv,  SvTrajectoryStock_[partitionIndex][k+1],  -deltaT);
			SveTrajectoryStock_[partitionIndex][k] = solveLTI<STATE_DIM>(_Gm, _Gve, SveTrajectoryStock_[partitionIndex][k+1], -deltaT);
			sTrajectoryStock_[partitionIndex][k]   = sTrajectoryStock_[partitionIndex][k+1] - deltaT * qTrajectoryStock_[partitionIndex][k];
		}

		_LvConstrained  = -RmInverseTrajectoryStock_[partitionIndex][k] *
				(RvConstrainedTrajectoryStock_[partitionIndex][k] + BmConstrainedTrajectoryStock_[partitionIndex][k].transpose()*SvTrajectoryStock_[partitionIndex][k]);
		_LveConstrained = -RmInverseTrajectoryStock_[partitionIndex][k] * BmConstrainedTrajectoryStock_[partitionIndex][k].transpose() * SveTrajectoryStock_[partitionIndex][k];

		// controller
		nominalControllersStock_[partitionIndex].k_[k]   = _LmConstrained - CmProjectedTrajectoryStock_[partitionIndex][k];
		nominalControllersStock_[partitionIndex].uff_[k] = nominalInputTrajectoriesStock_[partitionIndex][k]
		        - nominalControllersStock_[partitionIndex].k_[k]*nominalStateTrajectoriesStock_[partitionIndex][k]
				+ constraintStepSize * (_LveConstrained - EvProjectedTrajectoryStock_[partitionIndex][k]);
		nominalControllersStock_[partitionIndex].deltaUff_[k] = _LvConstrained;

	}

//	// set the final value for next Riccati equation
//	SmFinalStock_[partitionIndex]  = SmTrajectoryStock_[partitionIndex].front();
//	SvFinalStock_[partitionIndex]  = SvTrajectoryStock_[partitionIndex].front();
//	SveFinalStock_[partitionIndex] = SveTrajectoryStock_[partitionIndex].front();
//	sFinalStock_[partitionIndex]   = sTrajectoryStock_[partitionIndex].front();


	// testing the numerical stability
	if (settings_.checkNumericalStability_==true)
		for (int k=N-1; k>=0; k--) {
			// checking the numerical stability of the Riccati equations
			try {
				if (SmTrajectoryStock_[partitionIndex][k].hasNaN())   throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[partitionIndex][k].hasNaN())   throw std::runtime_error("Sv is unstable.");
				if (SveTrajectoryStock_[partitionIndex][k].hasNaN())  throw std::runtime_error("Sve is unstable.");
				if (sTrajectoryStock_[partitionIndex][k].hasNaN())    throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error) {
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[partitionIndex][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\n"<< SmTrajectoryStock_[partitionIndex][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SvTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "Sve[" << SsTimeTrajectoryStock_[partitionIndex][kp] << "]:\t"<< SveTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[partitionIndex][kp] << "]: \t"<< sTrajectoryStock_[partitionIndex][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}

			// checking the numerical stability of the controller parameters
			try {
				if (nominalControllersStock_[partitionIndex].k_[k].hasNaN())        throw std::runtime_error("Feedback gains are unstable.");
				if (nominalControllersStock_[partitionIndex].uff_[k].hasNaN())      throw std::runtime_error("uff gains are unstable.");
				if (nominalControllersStock_[partitionIndex].deltaUff_[k].hasNaN()) throw std::runtime_error("deltaUff is unstable.");
			}
			catch(const std::exception& error) {
				std::cerr << "what(): " << error.what() << " at time " << nominalControllersStock_[partitionIndex].time_[k] << " [sec]." << std::endl;
				exit(0);
			}
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
template <typename Derived>
bool SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

	if (squareMatrix.rows() != squareMatrix.cols())
		throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

	Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix, Eigen::EigenvaluesOnly);
	Eigen::VectorXd lambda = eig.eigenvalues();

	bool hasNegativeEigenValue = false;
	for (size_t j=0; j<lambda.size() ; j++)
		if (lambda(j) < 0.0) {
			hasNegativeEigenValue = true;
			lambda(j) = 1e-6;
		}

	if (hasNegativeEigenValue) {
		eig.compute(squareMatrix, Eigen::ComputeEigenvectors);
		squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
	} else {
		squareMatrix = 0.5*(squareMatrix+squareMatrix.transpose()).eval();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateControllerUpdateMaxNorm(
		scalar_t& maxDeltaUffNorm,
		scalar_t& maxDeltaUeeNorm) {

	maxDeltaUffNorm = 0.0;
	maxDeltaUeeNorm = 0.0;
	for (size_t i=initActivePartition_; i<=finalActivePartition_; i++)  {

		nominalStateFunc_[0].reset();
		nominalStateFunc_[0].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc_[0].setData( &(nominalStateTrajectoriesStock_[i]) );

		nominalInputFunc_[0].reset();
		nominalInputFunc_[0].setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		nominalInputFunc_[0].setData( &(nominalInputTrajectoriesStock_[i]) );

		for (size_t k=0; k<nominalControllersStock_[i].time_.size(); k++)  {

			maxDeltaUffNorm = std::max(maxDeltaUffNorm, nominalControllersStock_[i].deltaUff_[k].norm());

			state_vector_t nominalState;
			nominalStateFunc_[0].interpolate(nominalControllersStock_[i].time_[k], nominalState);
			size_t greatestLessTimeStampIndex = nominalStateFunc_[0].getGreatestLessTimeStampIndex();
			input_vector_t nominalInput;
			nominalInputFunc_[0].interpolate(nominalControllersStock_[i].time_[k], nominalInput, greatestLessTimeStampIndex);
			input_vector_t deltaUee = nominalInput - nominalControllersStock_[i].k_[k]*nominalState - nominalControllersStock_[i].uff_[k];
			maxDeltaUeeNorm = std::max(maxDeltaUeeNorm, deltaUee.norm());

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::printString(const std::string& text) {
	std::cerr << text << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::printRolloutInfo()  {

	std::cerr << "optimization cost:         " << nominalTotalCost_ << std::endl;
	std::cerr << "constraint type-1 ISE:     " << nominalConstraint1ISE_ << std::endl;
	std::cerr << "constraint type-1 MaxNorm: " << nominalConstraint1MaxNorm_ << std::endl;
	std::cerr << "constraint type-2 ISE:     " << nominalConstraint2ISE_ << std::endl;
	std::cerr << "constraint type-2 MaxNorm: " << nominalConstraint2MaxNorm_ << std::endl;
	std::cerr << "final constraint type-2: 	 ";
	size_t itr = 0;
	for(size_t i=initActivePartition_; i<=finalActivePartition_; i++)
		for (size_t k=0; k<nc2FinalStock_[i].size(); k++) {
			std::cerr << "[" << itr  << "]: " << HvFinalStock_[i][k].head(nc2FinalStock_[i][k]).transpose() << ",  ";
			itr++;
		}
	std::cerr << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutLagrangeMultiplier(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock,
		std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock)  {

	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> constraint_vector_t;
	typedef Eigen::Matrix<double, Eigen::Dynamic, STATE_DIM> constraint_matrix_t;

	lagrangeTrajectoriesStock.resize(numPartitionings_);

	LinearInterpolation<constraint_vector_t, Eigen::aligned_allocator<constraint_vector_t> > vffFunc;
	LinearInterpolation<constraint_matrix_t, Eigen::aligned_allocator<constraint_matrix_t> > vfbFunc;

	for (size_t i=0; i<numPartitionings_; i++) {

		size_t N = timeTrajectoriesStock[i].size();
		lagrangeTrajectoriesStock[i].resize(N);

		// if the subsystem is not simulated (e.g. due to the initial time)
		if (N==0)  continue;

		vffFunc.reset();
		vffFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vffFunc.setData(&lagrangeMultiplierFunctionsStock[i].uff_);

		vfbFunc.reset();
		vfbFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vfbFunc.setData(&lagrangeMultiplierFunctionsStock[i].k_);

		for (size_t k=0; k<N; k++) {

			constraint_vector_t vff;
			vffFunc.interpolate(timeTrajectoriesStock[i][k], vff);
			size_t greatestLessTimeIndex = vffFunc.getGreatestLessTimeStampIndex();

			constraint_matrix_t vfb;
			vfbFunc.interpolate(timeTrajectoriesStock[i][k], vfb, greatestLessTimeIndex);

			lagrangeTrajectoriesStock[i][k] = vff + vfb*stateTrajectoriesStock[i][k];

		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCostate(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		state_vector_array2_t& costateTrajectoriesStock)  {

	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > nominalStateFunc;

	costateTrajectoriesStock.resize(numPartitionings_);

	for (size_t i=0; i<numPartitionings_; i++) {

		size_t N = timeTrajectoriesStock[i].size();
		costateTrajectoriesStock[i].resize(N);

		// if the subsystem is not simulated (e.g. due to the initial time)
		if (N==0) {
			costateTrajectoriesStock[i].clear();
			continue;
		}

		SmFunc.reset();
		SmFunc.setTimeStamp(&SsTimeTrajectoryStock_[i]);
		SmFunc.setData(&SmTrajectoryStock_[i]);
		SvFunc.reset();
		SvFunc.setTimeStamp(&SsTimeTrajectoryStock_[i]);
		SvFunc.setData(&SvTrajectoryStock_[i]);
		nominalStateFunc.reset();
		nominalStateFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		nominalStateFunc.setData(&nominalStateTrajectoriesStock_[i]);

		for (size_t k=0; k<N; k++) {

			const scalar_t& t = timeTrajectoriesStock[i][k];

			state_matrix_t Sm;
			SmFunc.interpolate(t, Sm);
			size_t greatestLessTimeStampIndex = SmFunc.getGreatestLessTimeStampIndex();
			state_vector_t Sv;
			SvFunc.interpolate(t, Sv, greatestLessTimeStampIndex);

			state_vector_t nominalState;
			nominalStateFunc.interpolate(t, nominalState);

			costateTrajectoriesStock[i][k] = Sv + Sm*(stateTrajectoriesStock[i][k]-nominalState);

		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateInputConstraintLagrangian(std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock) {

	// functions for controller and lagrane multiplier
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >     nominalStateFunc;

	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmInverseFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> >     RvFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> >     EvProjectedFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > CmProjectedFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmFunc;
	LinearInterpolation<control_constraint1_matrix_t,Eigen::aligned_allocator<control_constraint1_matrix_t> > DmDagerFunc;

	lagrangeMultiplierFunctionsStock.resize(numPartitionings_);

	for (size_t i=0; i<numPartitionings_; i++) {

		if (i<initActivePartition_ || i>finalActivePartition_) {
			lagrangeMultiplierFunctionsStock[i].clear();
			continue;
		}
		nominalStateFunc.reset();
		nominalStateFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc.setData( &(nominalStateTrajectoriesStock_[i]) );

		BmFunc.reset();
		BmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		BmFunc.setData( &(BmTrajectoryStock_[i]) );

		PmFunc.reset();
		PmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(PmTrajectoryStock_[i]) );

		RmInverseFunc.reset();
		RmInverseFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RmInverseFunc.setData( &(RmInverseTrajectoryStock_[i]) );

		RvFunc.reset();
		RvFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RvFunc.setData( &(RvTrajectoryStock_[i]) );

		EvProjectedFunc.reset();
		EvProjectedFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		EvProjectedFunc.setData( &(EvProjectedTrajectoryStock_[i]) );

		CmProjectedFunc.reset();
		CmProjectedFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		CmProjectedFunc.setData( &(CmProjectedTrajectoryStock_[i]) );

		RmFunc.reset();
		RmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RmFunc.setData( &(RmTrajectoryStock_[i]) );

		DmDagerFunc.reset();
		DmDagerFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		DmDagerFunc.setData( &(DmDagerTrajectoryStock_[i]) );

		size_t N = SsTimeTrajectoryStock_[i].size();

		lagrangeMultiplierFunctionsStock[i].time_ = SsTimeTrajectoryStock_[i];
		lagrangeMultiplierFunctionsStock[i].k_.resize(N);
		lagrangeMultiplierFunctionsStock[i].uff_.resize(N);
		lagrangeMultiplierFunctionsStock[i].deltaUff_.resize(N);

		for (size_t k=0; k<N; k++) {

			const double& time = SsTimeTrajectoryStock_[i][k];
			size_t greatestLessTimeStampIndex;

			state_vector_t nominalState;
			nominalStateFunc.interpolate(time, nominalState);
			greatestLessTimeStampIndex = nominalStateFunc.getGreatestLessTimeStampIndex();

			control_gain_matrix_t Bm;
			BmFunc.interpolate(time, Bm, greatestLessTimeStampIndex);
			control_feedback_t Pm;
			PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);
			input_vector_t Rv;
			RvFunc.interpolate(time, Rv, greatestLessTimeStampIndex);
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(time, RmInverse, greatestLessTimeStampIndex);
			input_vector_t EvProjected;
			EvProjectedFunc.interpolate(time, EvProjected, greatestLessTimeStampIndex);
			control_feedback_t CmProjected;
			CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);

			control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*SmTrajectoryStock_[i][k]);
			input_vector_t   Lv  = RmInverse * (Rv + Bm.transpose()*SvTrajectoryStock_[i][k]);
			input_vector_t   Lve = RmInverse * (Bm.transpose()*SveTrajectoryStock_[i][k]);

			const size_t& nc1 = nc1TrajectoriesStock_[i][greatestLessTimeStampIndex];

			control_constraint1_matrix_t DmDager;
			DmDagerFunc.interpolate(time, DmDager, greatestLessTimeStampIndex);
			control_matrix_t Rm;
			RmFunc.interpolate(time, Rm, greatestLessTimeStampIndex);

			Eigen::MatrixXd DmDagerTransRm = DmDager.leftCols(nc1).transpose() * Rm;

			lagrangeMultiplierFunctionsStock[i].k_[k]   = DmDagerTransRm * (CmProjected - Lm);
			lagrangeMultiplierFunctionsStock[i].uff_[k] = DmDagerTransRm * (EvProjected-Lv-Lve)
					- lagrangeMultiplierFunctionsStock[i].k_[k]*nominalState;
			lagrangeMultiplierFunctionsStock[i].deltaUff_[k] = Eigen::VectorXd::Zero(nc1);

			// checking the numerical stability
			try {
				if (lagrangeMultiplierFunctionsStock[i].k_[k] != lagrangeMultiplierFunctionsStock[i].k_[k])
					throw std::runtime_error("Feedback lagrangeMultiplier is unstable.");
				if (lagrangeMultiplierFunctionsStock[i].uff_[k] != lagrangeMultiplierFunctionsStock[i].uff_[k])
					throw std::runtime_error("Feedforward lagrangeMultiplier is unstable.");
			}
			catch(const std::exception& error)  {
				std::cerr << "what(): " << error.what() << " at time " << lagrangeMultiplierFunctionsStock[i].time_[k] << " [sec]." << std::endl;
			}

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateMeritFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoryStock,
		const std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock,
		const scalar_t& totalCost,
		scalar_t& meritFunctionValue,
		scalar_t& constraintISE)  {

	// add cost function
	meritFunctionValue = totalCost;

	// add the L2 penalty for constraint violation
	calculateConstraintISE(timeTrajectoriesStock, nc1TrajectoriesStock, EvTrajectoryStock, constraintISE);
	double pho = 1.0;
	if (settings_.maxNumIterationsSLQ_>1)
		pho = (iteration_-1)/(settings_.maxNumIterationsSLQ_-1) * settings_.meritFunctionRho_;

	meritFunctionValue += 0.5*pho*constraintISE;

	// add the the lagrangian term for the constraint
	scalar_t currentIntermediateMerit;
	scalar_t nextIntermediateMerit;

	for (size_t i=0; i<numPartitionings_; i++)
	{
		// integrates the intermediate merit using the trapezoidal approximation method
		currentIntermediateMerit = 0.0;
		nextIntermediateMerit = 0.0;
		for (size_t k=0; k+1<timeTrajectoriesStock[i].size(); k++)
		{
			if (k==0)
				currentIntermediateMerit = EvTrajectoryStock[i][k].head(nc1TrajectoriesStock[i][k]).transpose() * lagrangeTrajectoriesStock[i][k];
			else
				currentIntermediateMerit = nextIntermediateMerit;

			nextIntermediateMerit = EvTrajectoryStock[i][k+1].head(nc1TrajectoriesStock[i][k+1]).transpose() * lagrangeTrajectoriesStock[i][k+1];

			meritFunctionValue += 0.5*(currentIntermediateMerit+nextIntermediateMerit)*(timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectories (std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
		state_vector_array2_t& nominalStateTrajectoriesStock,
		input_vector_array2_t& nominalInputTrajectoriesStock)   {

	nominalTimeTrajectoriesStock   = nominalTimeTrajectoriesStock_;
	nominalStateTrajectoriesStock  = nominalStateTrajectoriesStock_;
	nominalInputTrajectoriesStock  = nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& nominalTimeTrajectoriesStock,
		std::shared_ptr<state_vector_array2_t>& nominalStateTrajectoriesStock,
		std::shared_ptr<input_vector_array2_t>& nominalInputTrajectoriesStock)   {

	nominalTimeTrajectoriesStock   = std::make_shared<std::vector<scalar_array_t>>(nominalTimeTrajectoriesStock_);
	nominalStateTrajectoriesStock  = std::make_shared<state_vector_array2_t>(nominalStateTrajectoriesStock_);
	nominalInputTrajectoriesStock  = std::make_shared<input_vector_array2_t>(nominalInputTrajectoriesStock_);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findActiveSubsystemIndex(
		const scalar_array_t& partitioningTimes,
		const double& time) {

	int activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);

	if (activeSubsystemIndex < 0) {
		std::string mesg = "Given time is less than the strat time (i.e. givenTime < partitioningTimes.front()): "
				+ std::to_string(time) + " < " + std::to_string(partitioningTimes.front());
		throw std::runtime_error(mesg);
	}

	if (activeSubsystemIndex == partitioningTimes.size()-1) {
		std::string mesg = "Given time is greater than the final time (i.e. partitioningTimes.back() < givenTime): "
				+ std::to_string(partitioningTimes.back()) + " < " + std::to_string(time);
		throw std::runtime_error(mesg);
	}

	return (size_t)activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getValueFuntion (
		const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {

	size_t activeSubsystem = findActiveSubsystemIndex(partitioningTimes_, time);

	state_matrix_t Sm;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &SmTrajectoryStock_[activeSubsystem]);
	SmFunc.interpolate(time, Sm);
	size_t greatestLessTimeStampIndex = SmFunc.getGreatestLessTimeStampIndex();

	state_vector_t Sv;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &SvTrajectoryStock_[activeSubsystem]);
	SvFunc.interpolate(time, Sv, greatestLessTimeStampIndex);

	eigen_scalar_t s;
	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > sFunc(
			&SsTimeTrajectoryStock_[activeSubsystem], &sTrajectoryStock_[activeSubsystem]);
	sFunc.interpolate(time, s, greatestLessTimeStampIndex);

	state_vector_t xNominal;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > xNominalFunc(&nominalTimeTrajectoriesStock_[activeSubsystem], &nominalStateTrajectoriesStock_[activeSubsystem]);
	xNominalFunc.interpolate(time, xNominal);

	state_vector_t deltaX = state-xNominal;

	valueFuntion = (s + deltaX.transpose()*Sv + 0.5*deltaX.transpose()*Sm*deltaX).eval()(0);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getPerformanceIndeces(
		scalar_t& costFunction, scalar_t& constraint1ISE, scalar_t& constraint2ISE) {

	costFunction = nominalTotalCost_;
	constraint1ISE = nominalConstraint1ISE_;
	constraint2ISE = nominalConstraint2ISE_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getController(controller_array_t& controllersStock) const {

	controllersStock = nominalControllersStock_;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const {

	controllersStock = std::make_shared<controller_array_t>(nominalControllersStock_);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller_t& SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::controller(
		size_t index) const {

	return nominalControllersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::truncateConterller(
		const scalar_array_t& partitioningTimes,
		const double& initTime,
		controller_array_t& controllersStock,
		size_t& initActiveSubsystemIndex,
		controller_array_t& deletedcontrollersStock) {

	deletedcontrollersStock.resize(numPartitionings_);
	for (size_t i=0; i<numPartitionings_; i++)
		deletedcontrollersStock[i].clear();

	// finding the active subsystem index at initTime_
	initActiveSubsystemIndex = findActiveSubsystemIndex(partitioningTimes, initTime);

	// saving the deleting part and clearing controllersStock
	for (size_t i=0; i<initActiveSubsystemIndex; i++)
		deletedcontrollersStock[i].swap(controllersStock[i]);

	if (controllersStock[initActiveSubsystemIndex].time_.empty()==true)  return;

	// interpolating uff
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t> > uffFunc;
	uffFunc.setTimeStamp(&controllersStock[initActiveSubsystemIndex].time_);
	uffFunc.setData(&controllersStock[initActiveSubsystemIndex].uff_);
	input_vector_t uffInit;
	uffFunc.interpolate(initTime, uffInit);
	size_t greatestLessTimeStampIndex = uffFunc.getGreatestLessTimeStampIndex();

	// interpolating k
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > kFunc;
	kFunc.setTimeStamp(&controllersStock[initActiveSubsystemIndex].time_);
	kFunc.setData(&controllersStock[initActiveSubsystemIndex].k_);
	control_feedback_t kInit;
	kFunc.interpolate(initTime, kInit, greatestLessTimeStampIndex);

	// deleting the controller in the active subsystem for the subsystems before initTime
	if (greatestLessTimeStampIndex>0) {

		deletedcontrollersStock[initActiveSubsystemIndex].time_.resize(greatestLessTimeStampIndex+1);
		deletedcontrollersStock[initActiveSubsystemIndex].uff_.resize(greatestLessTimeStampIndex+1);
		deletedcontrollersStock[initActiveSubsystemIndex].k_.resize(greatestLessTimeStampIndex+1);
		for (size_t k=0; k<=greatestLessTimeStampIndex; k++) {
			deletedcontrollersStock[initActiveSubsystemIndex].time_[k] = controllersStock[initActiveSubsystemIndex].time_[k];
			deletedcontrollersStock[initActiveSubsystemIndex].uff_[k] = controllersStock[initActiveSubsystemIndex].uff_[k];
			deletedcontrollersStock[initActiveSubsystemIndex].k_[k] = controllersStock[initActiveSubsystemIndex].k_[k];
		}

		controllersStock[initActiveSubsystemIndex].time_.erase (
				controllersStock[initActiveSubsystemIndex].time_.begin(),
				controllersStock[initActiveSubsystemIndex].time_.begin()+greatestLessTimeStampIndex);
		controllersStock[initActiveSubsystemIndex].uff_.erase (
				controllersStock[initActiveSubsystemIndex].uff_.begin(),
				controllersStock[initActiveSubsystemIndex].uff_.begin()+greatestLessTimeStampIndex);
		controllersStock[initActiveSubsystemIndex].k_.erase (
				controllersStock[initActiveSubsystemIndex].k_.begin(),
				controllersStock[initActiveSubsystemIndex].k_.begin()+greatestLessTimeStampIndex);
	}

	controllersStock[initActiveSubsystemIndex].time_[0] = initTime;
	controllersStock[initActiveSubsystemIndex].uff_[0] = uffInit;
	controllersStock[initActiveSubsystemIndex].k_[0] = kInit;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::rewindOptimizer(const size_t& firstIndex, bool initRun/*=false*/) {

	if (initRun==true) {
		// do something
	}

	if (firstIndex > numPartitionings_)
		throw std::runtime_error("Index for rewind is greater than the current size");

	const size_t preservedLength = numPartitionings_ + 1 - firstIndex;
	for (size_t i=0; i<numPartitionings_+1; i++)
		if (i<preservedLength) {
			sFinalStock_[i] = sFinalStock_[firstIndex+i];
			SvFinalStock_[i] = SvFinalStock_[firstIndex+i];
			SveFinalStock_[i] = SveFinalStock_[firstIndex+i];
			SmFinalStock_[i] = SmFinalStock_[firstIndex+i];
			xFinalStock_[i] = xFinalStock_[firstIndex+i];
		} else {
			sFinalStock_[i].setZero();
			SvFinalStock_[i].setZero();
			SveFinalStock_[i].setZero();
			SmFinalStock_[i].setZero();
			xFinalStock_[i].setZero();
		}

	for (size_t i=0; i<numPartitionings_; i++)
		if (i<preservedLength-1)
			nominalControllersStock_[i].swap(nominalControllersStock_[firstIndex+i]);
		else
			nominalControllersStock_[i].clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer(const size_t& numPartitionings) {

	if (numPartitionings==0)
		throw std::runtime_error("Number of Partitionings cannot be zero!");

	/*
	 * Desired cost trajectories
	 */
	desiredTimeTrajectoryPtrStock_.resize(numPartitionings);
	desiredStateTrajectoryPtrStock_.resize(numPartitionings);
	desiredInputTrajectoryPtrStock_.resize(numPartitionings);

	/*
	 * Riccati solver variables and controller update
	 */
	sFinalStock_   = eigen_scalar_array_t(numPartitionings+1, eigen_scalar_t::Zero());
	SvFinalStock_  = state_vector_array_t(numPartitionings+1, state_vector_t::Zero());
	SveFinalStock_ = state_vector_array_t(numPartitionings+1, state_vector_t::Zero());
	SmFinalStock_  = state_matrix_array_t(numPartitionings+1, state_matrix_t::Zero());
	xFinalStock_   = state_vector_array_t(numPartitionings+1, state_vector_t::Zero());

	SsTimeTrajectoryStock_.resize(numPartitionings);
	SsNormalizedTimeTrajectoryStock_.resize(numPartitionings);
	SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitionings);
	sTrajectoryStock_.resize(numPartitionings);
	SvTrajectoryStock_.resize(numPartitionings);
	SveTrajectoryStock_.resize(numPartitionings);
	SmTrajectoryStock_.resize(numPartitionings);

	updateFeedForwardPoliciesStock_.resize(numPartitionings);

	/*
	 * approximate LQ variables
	 */
	AmTrajectoryStock_.resize(numPartitionings);
	BmTrajectoryStock_.resize(numPartitionings);

	nc1TrajectoriesStock_.resize(numPartitionings);
	EvTrajectoryStock_.resize(numPartitionings);
	CmTrajectoryStock_.resize(numPartitionings);
	DmTrajectoryStock_.resize(numPartitionings);
	nc2TrajectoriesStock_.resize(numPartitionings);
	HvTrajectoryStock_.resize(numPartitionings);
	FmTrajectoryStock_.resize(numPartitionings);

	qTrajectoryStock_.resize(numPartitionings);
	QvTrajectoryStock_.resize(numPartitionings);
	QmTrajectoryStock_.resize(numPartitionings);
	RvTrajectoryStock_.resize(numPartitionings);
	RmTrajectoryStock_.resize(numPartitionings);
	RmInverseTrajectoryStock_.resize(numPartitionings);
	PmTrajectoryStock_.resize(numPartitionings);

	nc2FinalStock_.resize(numPartitionings);
	HvFinalStock_.resize(numPartitionings);
	FmFinalStock_.resize(numPartitionings);
	qFinalStock_.resize(numPartitionings);
	QvFinalStock_.resize(numPartitionings);
	QmFinalStock_.resize(numPartitionings);

	// constraint type 1 coefficients
	DmDagerTrajectoryStock_.resize(numPartitionings);
	AmConstrainedTrajectoryStock_.resize(numPartitionings);
	QmConstrainedTrajectoryStock_.resize(numPartitionings);
	QvConstrainedTrajectoryStock_.resize(numPartitionings);
	EvProjectedTrajectoryStock_.resize(numPartitionings);
	CmProjectedTrajectoryStock_.resize(numPartitionings);
	DmProjectedTrajectoryStock_.resize(numPartitionings);
	if (settings_.useRiccatiSolver_==true) {
		RmConstrainedTrajectoryStock_.resize(numPartitionings);
	} else {
		BmConstrainedTrajectoryStock_.resize(numPartitionings);
		PmConstrainedTrajectoryStock_.resize(numPartitionings);
		RvConstrainedTrajectoryStock_.resize(numPartitionings);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setLogicRules(const LOGIC_RULES_T& logicRules) {

	logicRulesMachine_->setLogicRules(logicRules);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runInit() {

#ifdef BENCHMARK
	// Benchmarking
	static size_t nIterations = 0;
	static scalar_t tAvgLQ=0.0, tAvgBP=0.0, tAvgFP=0.0;
	nIterations++;
	auto start = std::chrono::steady_clock::now();
#endif

	// initial controller rollout
	rolloutTrajectory(initTime_, initState_, finalTime_,
			partitioningTimes_, nominalControllersStock_,
			nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
			nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_);

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvgFP = ((1.0 - 1.0/nIterations)* tAvgFP) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

	// to check convergence of the main loop, we need to compute the total cost and ISEs
	bool computePerformanceIndex = settings_.displayInfo_==true || settings_.maxNumIterationsSLQ_>1;
	if (computePerformanceIndex==true) {
		// calculate rollout constraint type-1 ISE
		nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
				nominalConstraint1ISE_);
		// calculate rollout constraint type-2 ISE
		if (settings_.noStateConstraints_==false)
			nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
					nominalConstraint2ISE_);
		else
			nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
		// calculate rollout cost
		calculateRolloutCost(nominalTimeTrajectoriesStock_, nominalEventsPastTheEndIndecesStock_,
				nominalStateTrajectoriesStock_, nominalInputTrajectoriesStock_,
				nominalTotalCost_);
	} else {
		nominalTotalCost_ = 0.0;
		nominalConstraint1ISE_ = nominalConstraint1MaxNorm_ = 0.0;
		nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
	}

#ifdef BENCHMARK
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	tAvgLQ = ((1.0 - 1.0/nIterations)* tAvgLQ) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
	// calculate controller
	if (settings_.useRiccatiSolver_==true)
		calculateController();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvgBP = ((1.0 - 1.0/nIterations)* tAvgBP) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif

	// display
	if (settings_.displayInfo_)
		printRolloutInfo();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runIteration() {

#ifdef BENCHMARK
	// Benchmarking
	nIterations++;
	auto start = std::chrono::steady_clock::now();
#endif

	bool computeISEs = settings_.displayInfo_==true || settings_.noStateConstraints_==false;

	// finding the optimal learningRate
	maxLearningRate_ = settings_.maxLearningRateGSLQP_;
	lineSearch(computeISEs);

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvgFP = ((1.0 - 1.0/nIterations)* tAvgFP) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

	// to check convergence of the main loop, we need to compute ISEs
	if (computeISEs==false) {
		// calculate constraint type-1 ISE and maximum norm
		nominalConstraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_,
				nominalConstraint1ISE_);
		// calculates type-2 constraint ISE and maximum norm
		if (settings_.noStateConstraints_==false)
		nominalConstraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_,
				nominalConstraint2ISE_);
		else
			nominalConstraint2ISE_ = nominalConstraint2MaxNorm_ = 0.0;
	}

#ifdef BENCHMARK
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	tAvgLQ = ((1.0 - 1.0/nIterations)* tAvgLQ) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	solveSequentialRiccatiEquations(SmHeuristics_, SvHeuristics_, sHeuristics_);
	// calculate controller
	if (settings_.useRiccatiSolver_==true)
		calculateController();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvgBP = ((1.0 - 1.0/nIterations)* tAvgBP) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif

	// display
	if (settings_.displayInfo_)
		printRolloutInfo();
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock /*=scalar_array2_t()*/,
		const state_vector_array2_t& desiredStateTrajectoriesStock /*=state_vector_array2_t()*/,
		const input_vector_array2_t& desiredInputTrajectoriesStock /*=input_vector_array2_t()*/)  {

	const controller_array_t noInitialController(partitioningTimes.size()-1, controller_t());

	// call the "run" method which uses the internal controllers stock (i.e. nominalControllersStock_)
	run(initTime, initState, finalTime, partitioningTimes, noInitialController,
			desiredTimeTrajectoriesStock, desiredStateTrajectoriesStock, desiredInputTrajectoriesStock);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void SLQ_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run(
		const scalar_t& initTime, const state_vector_t& initState, const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes,
		const controller_array_t& controllersStock,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock /*=scalar_array2_t()*/,
		const state_vector_array2_t& desiredStateTrajectoriesStock /*=state_vector_array2_t()*/,
		const input_vector_array2_t& desiredInputTrajectoriesStock /*=input_vector_array2_t()*/)  {

	// update numPartitionings_ if it has been changed
	if (numPartitionings_+1 != partitioningTimes.size()) {
		numPartitionings_  = partitioningTimes.size()-1;
		partitioningTimes_ = partitioningTimes;
		setupOptimizer(numPartitionings_);
	}

	// update partitioningTimes_
	partitioningTimes_ = partitioningTimes;

	// use nominalControllersStock_ if no initial controller is given
	if (controllersStock.empty()==false) {
		nominalControllersStock_ = controllersStock;
		if (controllersStock.size() != numPartitionings_)
			throw std::runtime_error("controllersStock has less controllers than the number of partitions.");
	} else {
		if (nominalControllersStock_.size() != numPartitionings_)
			throw std::runtime_error("The internal controller is not compatible with the number of partitions.");
	}

	// set desired trajectories for cost
	if (desiredTimeTrajectoriesStock.empty()==false) {

		if (desiredTimeTrajectoriesStock.size() != numPartitionings_)
			throw std::runtime_error("desiredTimeTrajectoriesStock has less elements than the number of partitions.");
		if (desiredStateTrajectoriesStock.size() != numPartitionings_)
			throw std::runtime_error("desiredStateTrajectoriesStock has less elements than the number of partitions.");
		if (desiredInputTrajectoriesStock.size() != numPartitionings_ && desiredInputTrajectoriesStock.empty()==false)
			throw std::runtime_error("desiredInputTrajectoriesStock has less elements than the number of partitions.");

		for (size_t i=0; i<numPartitionings_; i++) {

			desiredTimeTrajectoryPtrStock_[i]  = &desiredTimeTrajectoriesStock[i];
			desiredStateTrajectoryPtrStock_[i] = &desiredStateTrajectoriesStock[i];
			if (desiredInputTrajectoriesStock.empty()==false)
				desiredInputTrajectoryPtrStock_[i] = &desiredInputTrajectoriesStock[i];
			else
				desiredInputTrajectoryPtrStock_[i] = nullptr;
		} // end of i loop

	} else {
		if (settings_.displayInfo_) std::cerr << "WARNING: Desired trajectories are not provided." << std::endl;
		std::fill(desiredTimeTrajectoryPtrStock_.begin(), desiredTimeTrajectoryPtrStock_.end(), nullptr);
		std::fill(desiredStateTrajectoryPtrStock_.begin(), desiredStateTrajectoryPtrStock_.end(), nullptr);
		std::fill(desiredInputTrajectoryPtrStock_.begin(), desiredInputTrajectoryPtrStock_.end(), nullptr);
	}

	// update the LOGIC_RULES in the begining of the run routine and adjust the nominal controllerStock based on an user-defined function
	logicRulesMachine_.updateLogicRules(partitioningTimes_, nominalControllersStock_);

	// display
	if (settings_.displayInfo_) {
		std::cerr << std::endl << "SLQ solver starts from initial time " << initTime << " to final time " << finalTime << ".";
		logicRulesMachine_.displaySwitchedSystemsDistribution();
	}

	iteration_ = 0;
	initState_ = initState;
	initTime_  = initTime;
	finalTime_ = finalTime;

	iterationCost_.clear();
	iterationISE1_.clear();
	iterationISE2_.clear();

	// finding the initial active partition index and truncating the controller
	truncateConterller(partitioningTimes_, initTime_, nominalControllersStock_, initActivePartition_, deletedcontrollersStock_);

	// the final active partition index.
	finalActivePartition_ = findActiveSubsystemIndex(partitioningTimes_, finalTime_);

	// display
	if (settings_.displayInfo_)
		std::cerr << "\n#### Iteration " << iteration_ << " (Dynamics might have been violated)" << std::endl;

	// if a controller is not set for a partition prevent feedforwrd policy to be updated for that partition
	for (size_t i=0; i<numPartitionings_; i++)
		updateFeedForwardPoliciesStock_[i] = nominalControllersStock_[i].time_.empty() ? false : true;

	// run loop initializer and update the member variables
	runInit();

	// after iteration zero always allow feedforward policy update
	for (size_t i=0; i<numPartitionings_; i++)
		updateFeedForwardPoliciesStock_[i] = true;

	iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
	iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );
	iterationISE2_.push_back( (Eigen::VectorXd(1) << nominalConstraint2ISE_).finished() );

	// convergence conditions varaibles
	scalar_t relCost;
	scalar_t relConstraint1ISE;
	bool isLearningRateStarZero = false;
	bool isCostFunctionConverged = false;
	bool isConstraint1Satisfied  = false;
	bool isOptimizationConverged = false;

	// SLQ main loop
	while (iteration_+1<settings_.maxNumIterationsSLQ_ && isOptimizationConverged==false)  {

		// increament iteration counter
		iteration_++;

		scalar_t costCashed = nominalTotalCost_;
		scalar_t constraint1ISECashed = nominalConstraint1ISE_;

		// display
		if (settings_.displayInfo_)  std::cerr << "\n#### Iteration " << iteration_ << std::endl;

		// run the an itration of the SLQ algorithm and update the member variables
		runIteration();

		iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
		iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );
		iterationISE2_.push_back( (Eigen::VectorXd(1) << nominalConstraint2ISE_).finished() );

		// loop break variables
		relCost = std::abs(nominalTotalCost_-costCashed);
		relConstraint1ISE = std::abs(nominalConstraint1ISE_-constraint1ISECashed);
		isConstraint1Satisfied  = nominalConstraint1ISE_<=settings_.minAbsConstraint1ISE_ || relConstraint1ISE<=settings_.minRelConstraint1ISE_;
		isLearningRateStarZero  = learningRateStar_==0;
		isCostFunctionConverged = relCost<=settings_.minRelCostGSLQP_ || isLearningRateStarZero;
		isOptimizationConverged = isCostFunctionConverged==true && isConstraint1Satisfied==true;

	}  // end of while loop

	if (settings_.displayInfo_)  std::cerr << "\n#### Final rollout" << std::endl;

#ifdef BENCHMARK
	nIterations++;
	auto start = std::chrono::steady_clock::now();
#endif

	bool computeISEs = settings_.noStateConstraints_==false ||
			settings_.displayInfo_==true || settings_.displayShortSummary_==true;

	// finding the final optimal learningRate and getting the optimal trajectories and controller
	maxLearningRate_ = settings_.maxLearningRateGSLQP_;
	lineSearch(computeISEs);

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvgFP = ((1.0 - 1.0/nIterations)* tAvgFP) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif

	/*
	 * solve Sequential Riccati Equations with learningRate 0.0,
	 * calculate the nominal co-state,
	 * add the deleted controller parts
	 */
//	runExit();

	// adding the deleted controller parts
	for (size_t i=0; i<initActivePartition_; i++)
		nominalControllersStock_[i].swap(deletedcontrollersStock_[i]);

	if (deletedcontrollersStock_[initActivePartition_].time_.empty()==false) {

		nominalControllersStock_[initActivePartition_].swap(deletedcontrollersStock_[initActivePartition_]);

		for (size_t k=0; k<deletedcontrollersStock_[initActivePartition_].time_.size(); k++) {
			nominalControllersStock_[initActivePartition_].time_.push_back(deletedcontrollersStock_[initActivePartition_].time_[k]);
			nominalControllersStock_[initActivePartition_].uff_.push_back(deletedcontrollersStock_[initActivePartition_].uff_[k]);
			nominalControllersStock_[initActivePartition_].k_.push_back(deletedcontrollersStock_[initActivePartition_].k_[k]);
		}
	}

	// display
	if (settings_.displayInfo_  || settings_.displayShortSummary_)  {

		std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr <<   "++++++++ SLQ solver is terminated +++++++" << std::endl;
		std::cerr <<   "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr << "Number of Iterations:      " <<  iteration_+1 << " out of " << settings_.maxNumIterationsSLQ_ << std::endl;

		printRolloutInfo();

		if (isOptimizationConverged==true) {
			if (isLearningRateStarZero==true)
				std::cerr << "SLQ successfully terminates as learningRate reduced to zero." << std::endl;
			else
				std::cerr << "SLQ successfully terminates as cost relative change (relCost=" << relCost <<") reached to the minimum value." << std::endl;

			if (nominalConstraint1ISE_<=settings_.minAbsConstraint1ISE_)
				std::cerr << "Type-1 constraint absolute ISE (absConstraint1ISE=" << nominalConstraint1ISE_ << ") reached to the minimum value." << std::endl;
			else
				std::cerr << "Type-1 constraint relative ISE (relConstraint1ISE=" << relConstraint1ISE << ") reached to the minimum value." << std::endl;
		} else {
			std::cerr << "Maximum number of iterations has reached." << std::endl;
		}
		std::cerr << std::endl;
	}

}

}  // ocs2 namespace



