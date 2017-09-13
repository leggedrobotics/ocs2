/*
 * Implementation of SLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod, markus
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
SLQP<STATE_DIM, INPUT_DIM>::~SLQP()
{
#ifdef BENCHMARK
	std::cout<<"Avg time for approximateOptimalControlProblem :  "<< tAvg1<<" ms "<< std::endl;
	std::cout<<"Avg time for riccati :  "<< tAvg2<<" ms "<< std::endl;
	std::cout<<"Avg time for linearsearch :  "<< tAvg3<<" ms "<< std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * Forward integrate the system dynamics with given controller:
 * 		inputs:
 * 			+ initTime: intial time
 * 			+ initState: initial state at time initTime
 * 			+ controllersStock: controller for each subsystem
 * 		outputs:
 * 			+ timeTrajectoriesStock:  rollout simulated time steps
 * 			+ stateTrajectoriesStock: rollout states
 * 			+ inputTrajectoriesStock: rollout control inputs
 * 			+ (optional) stateTrajectoriesStock_: rollout outputs
 * 			+ (optional) nc1TrajectoriesStock: number of active constraints at each time step
 * 			+ (optional) EvTrajectoryStock: value of the constraint (if the rollout is constrained the value is
 * 											always zero otherwise it is nonzero)
 */

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::rollout(const double& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock)  {

	if (controllersStock.size() != BASE::numSubsystems_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(BASE::numSubsystems_);
	stateTrajectoriesStock.resize(BASE::numSubsystems_);
	inputTrajectoriesStock.resize(BASE::numSubsystems_);

	// finding the active subsystem index at initTime
	size_t initActiveSubsystemIndex = this->findActiveSubsystemIndex(BASE::switchingTimes_, initTime);
	// finding the active subsystem index at initTime
	size_t finalActiveSubsystemIndex = this->findActiveSubsystemIndex(BASE::switchingTimes_, finalTime);

	double t0 = initTime;
	state_vector_t x0 = initState;
	double tf;
	for (int i=0; i<BASE::numSubsystems_; i++) {

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
		size_t maxNumSteps = BASE::options_.maxNumStepsPerSecond_ * std::max(1.0, BASE::switchingTimes_[i+1]-t0);

		// initialize subsystem i
		subsystemDynamicsPtrStock_[i]->initializeModel(BASE::systemStockIndexes_, BASE::switchingTimes_, x0, i, "GSLPQ");

		// final time
		tf = (i != finalActiveSubsystemIndex) ? BASE::switchingTimes_[i+1] : finalTime;

		// set controller for subsystem i
		if (controllersStock[i].time_.empty()==false) {
			subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);

		} else {

			if (BASE::options_.dispayGSLQP_)  std::cout << "LQP controller is used at period: [" << t0 << ", " << tf << "]" << std::endl;

			controller_t lqpPolicy = BASE::lqpControllersStock_[BASE::systemStockIndexes_[i]];
			double timeShift = t0-lqpPolicy.time_[0];
			for (size_t k=0; k<lqpPolicy.time_.size(); k++)
				lqpPolicy.time_[k] += timeShift;

			subsystemDynamicsPtrStock_[i]->setController(lqpPolicy);
		}

		// simulate subsystem i
		subsystemSimulatorsStockPtr_[i]->integrate(x0, t0, tf,
				stateTrajectoriesStock[i], timeTrajectoriesStock[i],
				1e-3, BASE::options_.AbsTolODE_, BASE::options_.RelTolODE_, maxNumSteps);

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::rollout(const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		state_vector_t& finalState,
		control_vector_t& finalInput,
		size_t& finalActiveSubsystemIndex)  {

	if (controllersStock.size() != BASE::numSubsystems_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	scalar_array_t timeTrajectory;
	state_vector_array_t stateTrajectory;

	// finding the active subsystem index at initTime and final time
	size_t initActiveSubsystemIndex = this->findActiveSubsystemIndex(BASE::switchingTimes_, initTime);
	finalActiveSubsystemIndex = this->findActiveSubsystemIndex(BASE::switchingTimes_, finalTime);

	double t0 = initTime;
	state_vector_t x0 = initState;
	double tf;
	for (int i=initActiveSubsystemIndex; i<finalActiveSubsystemIndex+1; i++) {

		timeTrajectory.clear();
		stateTrajectory.clear();

		// max number of steps of integration
		size_t maxNumSteps = BASE::options_.maxNumStepsPerSecond_ * std::max( 1.0, BASE::switchingTimes_[i+1]-t0);

		// initialize subsystem i
		subsystemDynamicsPtrStock_[i]->initializeModel(BASE::systemStockIndexes_, BASE::switchingTimes_, x0, i, "GSLPQ");
		// set controller for subsystem i
		subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);

		// simulate subsystem i
		tf = (i != finalActiveSubsystemIndex) ? BASE::switchingTimes_[i+1] : finalTime;
		subsystemSimulatorsStockPtr_[i]->integrate(x0, t0, tf,
				stateTrajectory, timeTrajectory,
				1e-3, BASE::options_.AbsTolODE_, BASE::options_.RelTolODE_, maxNumSteps);

		if (stateTrajectory.back() != stateTrajectory.back())
			throw std::runtime_error("System became unstable during the SLQP rollout.");

		// reset the initial time and state
		t0 = timeTrajectory.back();
		x0 = stateTrajectory.back();
	}

	// compute state, control and output
	finalState = stateTrajectory.back();
	subsystemDynamicsPtrStock_[finalActiveSubsystemIndex]->computeInput(timeTrajectory.back(), stateTrajectory.back(), finalInput);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::rollout(const double& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock,
		std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
		constraint1_vector_array2_t& EvTrajectoryStock,
		std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
		constraint2_vector_array2_t& HvTrajectoryStock,
		std::vector<size_t>& nc2FinalStock,
		constraint2_vector_array_t& HvFinalStock) {

	// STEP1 : perform normal rollout
	rollout(initTime, initState, finalTime, controllersStock,
			timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);

	// STEP2 : calculate constraint violations
	// constraint type 1 computations which consists of number of active constraints at each time point
	// and the value of the constraint (if the rollout is constrained the value is always zero otherwise
	// it is nonzero)
	nc1TrajectoriesStock.resize(BASE::numSubsystems_);
	EvTrajectoryStock.resize(BASE::numSubsystems_);

	// constraint type 2 computations which consists of number of active constraints at each time point
	// and the value of the constraint
	nc2TrajectoriesStock.resize(BASE::numSubsystems_);
	HvTrajectoryStock.resize(BASE::numSubsystems_);
	nc2FinalStock.resize(BASE::numSubsystems_);
	HvFinalStock.resize(BASE::numSubsystems_);


	for (int i=0; i<BASE::numSubsystems_; i++)
	{
		size_t N = timeTrajectoriesStock[i].size();
		nc1TrajectoriesStock[i].resize(N);
		EvTrajectoryStock[i].resize(N);
		nc2TrajectoriesStock[i].resize(N);
		HvTrajectoryStock[i].resize(N);

		// compute constraint1 trajectory for subsystem i
		for (int k=0; k<N; k++) {

			// constraint 1 type
			subsystemDynamicsPtrStock_[i]->computeConstriant1(timeTrajectoriesStock[i][k],
					stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k],
					nc1TrajectoriesStock[i][k], EvTrajectoryStock[i][k]);

			if (nc1TrajectoriesStock[i][k] > INPUT_DIM)
				throw std::runtime_error("Number of active type-1 constraints should be less-equal to the number of input dimension.");

			// constraint type 2
			subsystemDynamicsPtrStock_[i]->computeConstriant2(timeTrajectoriesStock[i][k],
					stateTrajectoriesStock[i][k],
					nc2TrajectoriesStock[i][k], HvTrajectoryStock[i][k]);

			if (nc2TrajectoriesStock[i][k] > INPUT_DIM)
				throw std::runtime_error("Number of active type-2 constraints should be less-equal to the number of input dimension.");

		}  // end of k loop

		// if the subsystem is not simulated (e.g. due to the initial time)
		if (N==0)
			nc2FinalStock[i] = 0;
		else
			subsystemDynamicsPtrStock_[i]->computeFinalConstriant2(timeTrajectoriesStock[i].back(), stateTrajectoriesStock[i].back(),
					nc2FinalStock[i], HvFinalStock[i]);

		if (nc2FinalStock[i] > INPUT_DIM)
			throw std::runtime_error("Number of active type-2 constraints at final time should be less-equal to the number of input dimension.");

	}  // end of i loop

}

/******************************************************************************************************/
/******************************************************************************************************/
/*****************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::calculateCostFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		const std::vector<std::vector<size_t> >& nc2TrajectoriesStock,
		const constraint2_vector_array2_t& HvTrajectoryStock,
		const std::vector<size_t>& nc2FinalStock,
		const constraint2_vector_array_t& HvFinalStock,
		scalar_t& totalCost) {

	calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock, totalCost);
	double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);

	for (int i=0; i<BASE::numSubsystems_; i++) {
		// integrates constraint type 2
		for (int k=0; k+1<timeTrajectoriesStock[i].size(); k++) {
			size_t nc2 = nc2TrajectoriesStock[i][k];
			if (nc2 > 0) {
				double dt = timeTrajectoriesStock[i][k+1]-timeTrajectoriesStock[i][k];
				totalCost += 0.5 * dt * stateConstraintPenalty * HvTrajectoryStock[i][k].head(nc2).squaredNorm();
			}
		}  // end of k loop

		// final constraint type 2
		size_t nc2Final = nc2FinalStock[i];
		if (nc2Final>0)
			totalCost += 0.5 * stateConstraintPenalty * HvFinalStock[i].head(nc2Final).squaredNorm();

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * compute the cost for a given rollout
 * 		inputs:
 * 			+ timeTrajectoriesStock:  rollout simulated time steps
 * 			+ stateTrajectoriesStock: rollout outputs
 * 			+ inputTrajectoriesStock: rollout control inputs
 *
 * 		outputs:
 * 			+ totalCost: the total cost of the trajectory
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::calculateCostFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

	totalCost = 0.0;
	for (int i=0; i<BASE::numSubsystems_; i++) {

		// integrates the intermediate cost using the trapezoidal approximation method
		scalar_t currentIntermediateCost;
		scalar_t nextIntermediateCost;
		for (int k=0; k+1<timeTrajectoriesStock[i].size(); k++) {

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
		if (i==BASE::finalActiveSubsystem_)  {
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
/*
 * approximates the nonlinear problem as a linear-quadratic problem around the nominal
 * state and control trajectories. This method updates the following variables:
 *
 * 		+ linearized system model
 * 		+ dxdt = Am(t)x(t) + Bm(t)u(t)
 * 		+ s.t. Cm(t)x(t) + Dm(t)t(t) + Ev(t) = 0
 * 		+ BASE::AmTrajectoryStock_: Am matrix
 * 		+ BASE::BmTrajectoryStock_: Bm matrix
 * 		+ BASE::CmTrajectoryStock_: Cm matrix
 * 		+ BASE::DmTrajectoryStock_: Dm matrix
 * 		+ BASE::EvTrajectoryStock_: Ev vector
 *
 * 		+ quadratized intermediate cost function
 * 		+ intermediate cost: q(t) + 0.5 y(t)Qm(t)y(t) + y(t)'Qv(t) + u(t)'Pm(t)y(t) + 0.5u(t)'Rm(t)u(t) + u(t)'Rv(t)
 * 		+ BASE::qTrajectoryStock_:  q
 * 		+ BASE::QvTrajectoryStock_: Qv vector
 * 		+ BASE::QmTrajectoryStock_: Qm matrix
 * 		+ BASE::PmTrajectoryStock_: Pm matrix
 * 		+ BASE::RvTrajectoryStock_: Rv vector
 * 		+ BASE::RmTrajectoryStock_: Rm matrix
 * 		+ BASE::RmInverseTrajectoryStock_: inverse of Rm matrix
 *
 * 		+ as well as the constrained coefficients of
 * 			linearized system model
 * 			quadratized intermediate cost function
 * 			quadratized final cost
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem()  {

	BASE::AmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::BmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::CmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::DmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::FmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::qTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::QvTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::QmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::RvTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::RmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::RmInverseTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::PmTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::qFinalStock_.resize(BASE::numSubsystems_);
	BASE::QvFinalStock_.resize(BASE::numSubsystems_);
	BASE::QmFinalStock_.resize(BASE::numSubsystems_);
	BASE::FmFinalStock_.resize(BASE::numSubsystems_);

	for (int i=0; i<BASE::numSubsystems_; i++) {

		int N = BASE::nominalTimeTrajectoriesStock_[i].size();

		if (N > 0) {
			// initialize subsystem i dynamics derivatives
			subsystemDerivativesPtrStock_[i]->initializeModel(BASE::systemStockIndexes_, BASE::switchingTimes_,
					BASE::nominalStateTrajectoriesStock_[i].front(), i, "GSLPQ");
		}

		BASE::AmTrajectoryStock_[i].resize(N);
		BASE::BmTrajectoryStock_[i].resize(N);
		BASE::CmTrajectoryStock_[i].resize(N);
		BASE::DmTrajectoryStock_[i].resize(N);
		BASE::FmTrajectoryStock_[i].resize(N);

		BASE::qTrajectoryStock_[i].resize(N);
		BASE::QvTrajectoryStock_[i].resize(N);
		BASE::QmTrajectoryStock_[i].resize(N);
		BASE::RvTrajectoryStock_[i].resize(N);
		BASE::RmTrajectoryStock_[i].resize(N);
		BASE::RmInverseTrajectoryStock_[i].resize(N);
		BASE::PmTrajectoryStock_[i].resize(N);

		for (int k=0; k<N; k++)
		{
			// LINEARIZE SYSTEM DYNAMICS AND CONSTRAINTS
			subsystemDerivativesPtrStock_[i]->setCurrentStateAndControl(BASE::nominalTimeTrajectoriesStock_[i][k],
					BASE::nominalStateTrajectoriesStock_[i][k], BASE::nominalInputTrajectoriesStock_[i][k]);
			subsystemDerivativesPtrStock_[i]->getDerivativeState(BASE::AmTrajectoryStock_[i][k]);
			subsystemDerivativesPtrStock_[i]->getDerivativesControl(BASE::BmTrajectoryStock_[i][k]);

			// if constraint type 1 is active
			if (BASE::nc1TrajectoriesStock_[i][k] > 0) {
				subsystemDerivativesPtrStock_[i]->getConstraint1DerivativesState(BASE::CmTrajectoryStock_[i][k]);
				subsystemDerivativesPtrStock_[i]->getConstraint1DerivativesControl(BASE::DmTrajectoryStock_[i][k]);
			}

			// if constraint type 2 is active
			if (BASE::nc2TrajectoriesStock_[i][k] > 0) {
				subsystemDerivativesPtrStock_[i]->getConstraint2DerivativesState(BASE::FmTrajectoryStock_[i][k]);
			}

			// QUADRATIC APPROXIMATION TO THE COST FUNCTION
			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(BASE::nominalTimeTrajectoriesStock_[i][k],
					BASE::nominalStateTrajectoriesStock_[i][k], BASE::nominalInputTrajectoriesStock_[i][k]);
			subsystemCostFunctionsPtrStock_[i]->evaluate(BASE::qTrajectoryStock_[i][k](0));
			subsystemCostFunctionsPtrStock_[i]->stateDerivative(BASE::QvTrajectoryStock_[i][k]);
			subsystemCostFunctionsPtrStock_[i]->stateSecondDerivative(BASE::QmTrajectoryStock_[i][k]);
			subsystemCostFunctionsPtrStock_[i]->controlDerivative(BASE::RvTrajectoryStock_[i][k]);
			subsystemCostFunctionsPtrStock_[i]->controlSecondDerivative(BASE::RmTrajectoryStock_[i][k]);
			BASE::RmInverseTrajectoryStock_[i][k] = BASE::RmTrajectoryStock_[i][k].inverse();
			subsystemCostFunctionsPtrStock_[i]->stateControlDerivative(BASE::PmTrajectoryStock_[i][k]);

		} // end of k loop


		if (i==BASE::finalActiveSubsystem_)  {
			subsystemCostFunctionsPtrStock_[i]->terminalCost(BASE::qFinalStock_[i](0));
			subsystemCostFunctionsPtrStock_[i]->terminalCostStateDerivative(BASE::QvFinalStock_[i]);
			subsystemCostFunctionsPtrStock_[i]->terminalCostStateSecondDerivative(BASE::QmFinalStock_[i]);
			// making sure that Qm remains PSD
			this->makePSD(BASE::QmFinalStock_[i]);
		}
		else {
			BASE::qFinalStock_[i].setZero();
			BASE::QvFinalStock_[i].setZero();
			BASE::QmFinalStock_[i].setZero();
		}
	}


	// constraint type-2 coefficients
	for (int i=0; i<BASE::numSubsystems_; i++) {

		int N = BASE::nominalTimeTrajectoriesStock_[i].size();

		// constraint type-2 coefficients
		for (int k=0; k<N; k++) {

			double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);

			// constrained type-2 intermediate coefficients
			size_t nc2 = BASE::nc2TrajectoriesStock_[i][k];

			if (nc2 > 0) {

				//subsystemDerivativesPtrStock_[i]->getConstraint2DerivativesState(FmTrajectoryStock_[i][k]);

				BASE::qTrajectoryStock_[i][k]  += 0.5 * stateConstraintPenalty * BASE::HvTrajectoryStock_[i][k].head(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
				BASE::QvTrajectoryStock_[i][k] += stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::HvTrajectoryStock_[i][k].head(nc2);
				BASE::QmTrajectoryStock_[i][k] += stateConstraintPenalty * BASE::FmTrajectoryStock_[i][k].topRows(nc2).transpose() * BASE::FmTrajectoryStock_[i][k].topRows(nc2);
			}
		}  // end of k loop

		// constrained type-2 final coefficients
		if (BASE::nc2FinalStock_[i] > 0) {
			size_t nc2 = BASE::nc2FinalStock_[i];

			subsystemDerivativesPtrStock_[i]->getFinalConstraint2DerivativesState(BASE::FmFinalStock_[i]);

			double stateConstraintPenalty = BASE::options_.stateConstraintPenaltyCoeff_ * pow(BASE::options_.stateConstraintPenaltyBase_, BASE::iteration_);

			BASE::qFinalStock_[i]  += 0.5 * stateConstraintPenalty * BASE::HvFinalStock_[i].head(nc2).transpose() * BASE::HvFinalStock_[i].head(nc2);
			BASE::QvFinalStock_[i] += stateConstraintPenalty * BASE::FmFinalStock_[i].topRows(nc2).transpose() * BASE::HvFinalStock_[i].head(nc2);
			BASE::QmFinalStock_[i] += stateConstraintPenalty * BASE::FmFinalStock_[i].topRows(nc2).transpose() * BASE::FmFinalStock_[i].topRows(nc2);
		}
	} // end of i loop


	// constraint type 1 coefficients
	BASE::DmDagerTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::AmConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::QmConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::QvConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::EvProjectedTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::CmProjectedTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::DmProjectedTrajectoryStock_.resize(BASE::numSubsystems_);
	if (BASE::options_.useRiccatiSolver_==true) {
		BASE::RmConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
	} else {
		BASE::BmConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
		BASE::PmConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
		BASE::RvConstrainedTrajectoryStock_.resize(BASE::numSubsystems_);
	}

	for (int i=0; i<BASE::numSubsystems_; i++) {

		int N = BASE::nominalTimeTrajectoriesStock_[i].size();

		BASE::DmDagerTrajectoryStock_[i].resize(N);

		BASE::AmConstrainedTrajectoryStock_[i].resize(N);
		BASE::QmConstrainedTrajectoryStock_[i].resize(N);
		BASE::QvConstrainedTrajectoryStock_[i].resize(N);
		if (BASE::options_.useRiccatiSolver_==true) {
			BASE::RmConstrainedTrajectoryStock_[i].resize(N);
		} else {
			BASE::BmConstrainedTrajectoryStock_[i].resize(N);
			BASE::PmConstrainedTrajectoryStock_[i].resize(N);
			BASE::RvConstrainedTrajectoryStock_[i].resize(N);
		}

		BASE::EvProjectedTrajectoryStock_[i].resize(N);
		BASE::CmProjectedTrajectoryStock_[i].resize(N);
		BASE::DmProjectedTrajectoryStock_[i].resize(N);

		for (int k=0; k<N; k++) {
			size_t nc1 = BASE::nc1TrajectoriesStock_[i][k];

			if (nc1 == 0) {
				BASE::DmDagerTrajectoryStock_[i][k].setZero();
				BASE::EvProjectedTrajectoryStock_[i][k].setZero();
				BASE::CmProjectedTrajectoryStock_[i][k].setZero();
				BASE::DmProjectedTrajectoryStock_[i][k].setZero();

				BASE::AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k];
				BASE::QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k];
				BASE::QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k];
				if (BASE::options_.useRiccatiSolver_==true) {
					BASE::RmConstrainedTrajectoryStock_[i][k] = BASE::RmTrajectoryStock_[i][k];
				} else {
					BASE::BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k];
					BASE::PmConstrainedTrajectoryStock_[i][k] = BASE::PmTrajectoryStock_[i][k];
					BASE::RvConstrainedTrajectoryStock_[i][k] = BASE::RvTrajectoryStock_[i][k];
				}

			} else {
				Eigen::MatrixXd Cm = BASE::CmTrajectoryStock_[i][k].topRows(nc1);
				Eigen::MatrixXd Dm = BASE::DmTrajectoryStock_[i][k].topRows(nc1);
				Eigen::MatrixXd Ev = BASE::EvTrajectoryStock_[i][k].head(nc1);

				Eigen::MatrixXd RmProjected = ( Dm*BASE::RmInverseTrajectoryStock_[i][k]*Dm.transpose() ).inverse();
				Eigen::MatrixXd DmDager = BASE::RmInverseTrajectoryStock_[i][k] * Dm.transpose() * RmProjected;

				BASE::DmDagerTrajectoryStock_[i][k].leftCols(nc1) = DmDager;
				BASE::EvProjectedTrajectoryStock_[i][k] = DmDager * Ev;
				BASE::CmProjectedTrajectoryStock_[i][k] = DmDager * Cm;
				BASE::DmProjectedTrajectoryStock_[i][k] = DmDager * Dm;

				control_matrix_t DmNullSpaceProjection = control_matrix_t::Identity() - BASE::DmProjectedTrajectoryStock_[i][k];
				state_matrix_t   PmTransDmDagerCm = BASE::PmTrajectoryStock_[i][k].transpose()*BASE::CmProjectedTrajectoryStock_[i][k];

				BASE::AmConstrainedTrajectoryStock_[i][k] = BASE::AmTrajectoryStock_[i][k] - BASE::BmTrajectoryStock_[i][k]*BASE::CmProjectedTrajectoryStock_[i][k];
				BASE::QmConstrainedTrajectoryStock_[i][k] = BASE::QmTrajectoryStock_[i][k] + Cm.transpose()*RmProjected*Cm - PmTransDmDagerCm - PmTransDmDagerCm.transpose();
				BASE::QvConstrainedTrajectoryStock_[i][k] = BASE::QvTrajectoryStock_[i][k] - BASE::CmProjectedTrajectoryStock_[i][k].transpose()*BASE::RvTrajectoryStock_[i][k];
				if (BASE::options_.useRiccatiSolver_==true) {
					BASE::RmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::RmTrajectoryStock_[i][k] * DmNullSpaceProjection;
				} else {
					BASE::BmConstrainedTrajectoryStock_[i][k] = BASE::BmTrajectoryStock_[i][k] * DmNullSpaceProjection;
					BASE::PmConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::PmTrajectoryStock_[i][k];
					BASE::RvConstrainedTrajectoryStock_[i][k] = DmNullSpaceProjection.transpose() * BASE::RvTrajectoryStock_[i][k];
				}
			}

			// making sure that constrained Qm is PSD
			this->makePSD(BASE::QmConstrainedTrajectoryStock_[i][k]);

		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * calculates the controller and linear function approximation of the type-1 constraint Lagrangian:
 * 		This method uses the following variables:
 * 			+ constrained, linearized model
 * 			+ constrained, quadratized cost
 *
 * 		The method outputs:
 * 			+ BASE::nominalControllersStock_: the controller that stabilizes the system around the new nominal trajectory and
 * 								improves the constraints as well as the increment to the feedforward control input.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::calculateController() {

	// functions for controller and lagrane multiplier
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >     nominalStateFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > nominalInputFunc;

	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmInverseFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     RvFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     EvProjectedFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > CmProjectedFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     DmProjectedFunc;

	for (size_t i=0; i<BASE::numSubsystems_; i++) {

		if (i<BASE::initActiveSubsystem_ || i>BASE::finalActiveSubsystem_) {
			BASE::nominalControllersStock_[i].clear();
			continue;
		}

		// functions for controller and lagrane multiplier

		nominalStateFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc.setData( &(BASE::nominalStateTrajectoriesStock_[i]) );

		nominalInputFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		nominalInputFunc.setData( &(BASE::nominalInputTrajectoriesStock_[i]) );

		BmFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		BmFunc.setData( &(BASE::BmTrajectoryStock_[i]) );

		PmFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(BASE::PmTrajectoryStock_[i]) );

		RmInverseFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		RmInverseFunc.setData( &(BASE::RmInverseTrajectoryStock_[i]) );

		RvFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		RvFunc.setData( &(BASE::RvTrajectoryStock_[i]) );

		EvProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		EvProjectedFunc.setData( &(BASE::EvProjectedTrajectoryStock_[i]) );

		CmProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		CmProjectedFunc.setData( &(BASE::CmProjectedTrajectoryStock_[i]) );

		DmProjectedFunc.setTimeStamp( &(BASE::nominalTimeTrajectoriesStock_[i]) );
		DmProjectedFunc.setData( &(BASE::DmProjectedTrajectoryStock_[i]) );

		int N = BASE::SsTimeTrajectoryStock_[i].size();

		BASE::nominalControllersStock_[i].time_ = BASE::SsTimeTrajectoryStock_[i];
		BASE::nominalControllersStock_[i].k_.resize(N);
		BASE::nominalControllersStock_[i].uff_.resize(N);
		BASE::nominalControllersStock_[i].deltaUff_.resize(N);

		for (int k=0; k<N; k++) {

			const double& time = BASE::SsTimeTrajectoryStock_[i][k];
			size_t greatestLessTimeStampIndex;

			state_vector_t nominalState;
			nominalStateFunc.interpolate(time, nominalState);
			greatestLessTimeStampIndex = nominalStateFunc.getGreatestLessTimeStampIndex();
			control_vector_t nominalInput;
			nominalInputFunc.interpolate(time, nominalInput, greatestLessTimeStampIndex);

			control_gain_matrix_t Bm;
			BmFunc.interpolate(time, Bm, greatestLessTimeStampIndex);
			control_feedback_t Pm;
			PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);
			control_vector_t Rv;
			RvFunc.interpolate(time, Rv, greatestLessTimeStampIndex);
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(time, RmInverse, greatestLessTimeStampIndex);
			control_vector_t EvProjected;
			EvProjectedFunc.interpolate(time, EvProjected, greatestLessTimeStampIndex);
			control_feedback_t CmProjected;
			CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);
			control_matrix_t DmProjected;
			DmProjectedFunc.interpolate(time, DmProjected, greatestLessTimeStampIndex);

			control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*BASE::SmTrajectoryStock_[i][k]);
			control_vector_t   Lv  = RmInverse * (Rv + Bm.transpose()*BASE::SvTrajectoryStock_[i][k]);
			control_vector_t   Lve = RmInverse * (Bm.transpose()*BASE::SveTrajectoryStock_[i][k]);

			control_matrix_t DmNullProjection = control_matrix_t::Identity()-DmProjected;
			BASE::nominalControllersStock_[i].k_[k]   = -DmNullProjection*Lm - CmProjected;
			BASE::nominalControllersStock_[i].uff_[k] = nominalInput - BASE::nominalControllersStock_[i].k_[k]*nominalState
					- BASE::options_.constraintStepSize_ * (DmNullProjection*Lve + EvProjected);
			BASE::nominalControllersStock_[i].deltaUff_[k] = -DmNullProjection*Lv;

			// checking the numerical stability of the controller parameters
			if (BASE::options_.checkNumericalStability_==true) {
				try {
					if (BASE::nominalControllersStock_[i].k_[k] != BASE::nominalControllersStock_[i].k_[k])
						throw std::runtime_error("Feedback gains are unstable.");
					if (BASE::nominalControllersStock_[i].deltaUff_[k] != BASE::nominalControllersStock_[i].deltaUff_[k])
						throw std::runtime_error("feedForwardControl is unstable.");
				}
				catch(const std::exception& error)  {
					std::cerr << "what(): " << error.what() << " at time " << BASE::nominalControllersStock_[i].time_[k] << " [sec]." << std::endl;
				}
			}

		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/*
 * line search on the feedforwrd parts of the controller and lagrange multipliers.
 * Based on the option flag lineSearchByMeritFuntion_ it uses two different approaches for line search:
 * 		+ the constraint correction term is added by a user defined stepSize.
 * 		The line search uses the pure cost function for choosing the best stepSize.
 *
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::lineSearch(scalar_t& learningRateStar, scalar_t maxLearningRateStar/*=1.0*/)  {

	// display
	if (BASE::options_.dispayGSLQP_)  {
		// less-equal operator for eigen vectors
		auto eigenVectorLessEqual = [] (const control_vector_t& u1, const control_vector_t& u2){ return u1.norm() < u2.norm(); };

		control_vector_array_t maxDeltaUffStock(BASE::finalActiveSubsystem_-BASE::initActiveSubsystem_+1);
		for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)  {
			maxDeltaUffStock[i-BASE::initActiveSubsystem_]  = *std::max_element(BASE::nominalControllersStock_[i].deltaUff_.begin(), BASE::nominalControllersStock_[i].deltaUff_.template end(), eigenVectorLessEqual);
		}
		control_vector_t maxDeltaUff  = *std::max_element(maxDeltaUffStock.begin(), maxDeltaUffStock.end(), eigenVectorLessEqual);
		std::cerr << "max delta_uff norm: " << maxDeltaUff.norm()  << std::endl;
	}

	// perform one rollout while the input correction for the type-1 constraint is considered.
	rollout(BASE::initTime_, BASE::initState_, BASE::finalTime_, BASE::nominalControllersStock_, BASE::nominalTimeTrajectoriesStock_,
			BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nc2TrajectoriesStock_,
			BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);
	calculateCostFunction(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

	// display
	if (BASE::options_.dispayGSLQP_)  {
		this->calculateConstraintISE(BASE::nominalTimeTrajectoriesStock_, BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_, BASE::nominalConstraint1ISE_);

		std::cerr << "\t learningRate 0.0 \t cost: " << BASE::nominalTotalCost_ << " \t constraint ISE: " << BASE::nominalConstraint1ISE_ << std::endl;
		if (std::accumulate(BASE::nc2FinalStock_.begin(), BASE::nc2FinalStock_.end(), 0) > 0) {
			std::cerr << "\t final constraint type-2:   ";
			for(size_t i=0; i<BASE::numSubsystems_; i++) std::cerr << "[" << i  << "]: " << BASE::HvFinalStock_[i].head(BASE::nc2FinalStock_[i]).transpose() << ",  ";
			std::cerr << std::endl;
		}

	}
	scalar_t learningRate = maxLearningRateStar;
	const controller_array_t controllersStock = BASE::nominalControllersStock_;

	// local search forward simulation's variables
	scalar_t lsTotalCost;
	controller_array_t           		lsControllersStock(BASE::numSubsystems_);
	std::vector<scalar_array_t>         lsTimeTrajectoriesStock(BASE::numSubsystems_);
	state_vector_array2_t   			lsStateTrajectoriesStock(BASE::numSubsystems_);
	control_vector_array2_t 			lsInputTrajectoriesStock(BASE::numSubsystems_);
	std::vector<std::vector<size_t> >   lsNc1TrajectoriesStock(BASE::numSubsystems_);
	constraint1_vector_array2_t 		lsEvTrajectoryStock(BASE::numSubsystems_);
	std::vector<std::vector<size_t> >   lsNc2TrajectoriesStock(BASE::numSubsystems_);
	constraint2_vector_array2_t 		lsHvTrajectoryStock(BASE::numSubsystems_);
	std::vector<size_t>               	lsNc2FinalStock(BASE::numSubsystems_);
	constraint2_vector_array_t	 		lsHvFinalStock(BASE::numSubsystems_);

	while (learningRate >= BASE::options_.minLearningRateGSLQP_)  {
		// modifying uff by the local increments
		lsControllersStock = controllersStock;
		for (int i=0; i<BASE::numSubsystems_; i++)
			for (int k=0; k<lsControllersStock[i].time_.size(); k++)
				lsControllersStock[i].uff_[k] += learningRate*lsControllersStock[i].deltaUff_[k];

		// perform rollout
		try {
			rollout(BASE::initTime_, BASE::initState_, BASE::finalTime_, lsControllersStock, lsTimeTrajectoriesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock,
					lsNc1TrajectoriesStock, lsEvTrajectoryStock,  lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock);
			// calculate rollout cost
			calculateCostFunction(lsTimeTrajectoriesStock, lsStateTrajectoriesStock, lsInputTrajectoriesStock, lsNc2TrajectoriesStock, lsHvTrajectoryStock, lsNc2FinalStock, lsHvFinalStock, lsTotalCost);

			// display
			if (BASE::options_.dispayGSLQP_) {
				scalar_t lsConstraint1ISE;
				this->calculateConstraintISE(lsTimeTrajectoriesStock, lsNc1TrajectoriesStock, lsEvTrajectoryStock, lsConstraint1ISE);
				std::cerr << "\t learningRate " << learningRate << " \t cost: " << lsTotalCost << " \t constraint ISE: " << lsConstraint1ISE << std::endl;
			}
		}
		catch(const std::exception& error)
		{
			std::cerr << "\t rollout with learningRate " << learningRate << " is terminated due to the slow simulation!" << std::endl;
			lsTotalCost  = std::numeric_limits<scalar_t>::max();
		}

		// break condition 1: it exits with largest learningRate that its cost is smaller than nominal cost.
		if (lsTotalCost < BASE::nominalTotalCost_*(1-1e-3*learningRate))
			break;  // exit while loop
		else
			learningRate = 0.5*learningRate;

	}  // end of while


	if (learningRate >= BASE::options_.minLearningRateGSLQP_)  {
		BASE::nominalTotalCost_      = lsTotalCost;
		learningRateStar = learningRate;

		BASE::nc2FinalStock_.swap(lsNc2FinalStock);
		BASE::HvFinalStock_.swap(lsHvFinalStock);

		for (size_t i = 0; i<BASE::numSubsystems_; i++)	// swapping where possible for efficiency
		{
			BASE::nominalControllersStock_[i].swap(lsControllersStock[i]);
			BASE::nominalTimeTrajectoriesStock_[i].swap(lsTimeTrajectoriesStock[i]);
			BASE::nominalStateTrajectoriesStock_[i].swap(lsStateTrajectoriesStock[i]);
			BASE::nominalInputTrajectoriesStock_[i].swap(lsInputTrajectoriesStock[i]);
			BASE::nc1TrajectoriesStock_[i].swap(lsNc1TrajectoriesStock[i]);
			BASE::EvTrajectoryStock_[i].swap(lsEvTrajectoryStock[i]);
			BASE::nc2TrajectoriesStock_[i].swap(lsNc2TrajectoriesStock[i]);
			BASE::HvTrajectoryStock_[i].swap(lsHvTrajectoryStock[i]);
		}

	} else // since the open loop input is not change, the nominal trajectories will be unchanged
		learningRateStar = 0.0;

	// clear the feedforward increments
	for (size_t i=0; i<BASE::numSubsystems_; i++)
		BASE::nominalControllersStock_[i].deltaUff_.clear();

	// display
	if (BASE::options_.dispayGSLQP_)  std::cerr << "The chosen learningRate is: " << learningRateStar << std::endl;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::solveSequentialRiccatiEquations(const scalar_t& learningRate,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal, const eigen_scalar_t& sFinal)  {

	BASE::SsTimeTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::SsNormalizedTimeTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::sTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::SvTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::SveTrajectoryStock_.resize(BASE::numSubsystems_);
	BASE::SmTrajectoryStock_.resize(BASE::numSubsystems_);

	BASE::SmFinalStock_[BASE::finalActiveSubsystem_+1]  = SmFinal;
	BASE::SvFinalStock_[BASE::finalActiveSubsystem_+1]  = SvFinal;
	BASE::SveFinalStock_[BASE::finalActiveSubsystem_+1] = state_vector_t::Zero();
	BASE::sFinalStock_[BASE::finalActiveSubsystem_+1]   = sFinal;

	for (int i=BASE::numSubsystems_-1; i>=0; i--) {

		if (i<BASE::initActiveSubsystem_ || BASE::finalActiveSubsystem_<i) {

			BASE::SsTimeTrajectoryStock_[i].clear();
			BASE::SmTrajectoryStock_[i].clear();
			BASE::SvTrajectoryStock_[i].clear();
			BASE::SveTrajectoryStock_[i].clear();
			BASE::sTrajectoryStock_[i].clear();

			BASE::SmFinalStock_[i].setZero();
			BASE::SvFinalStock_[i].setZero();
			BASE::SveFinalStock_[i].setZero();
			BASE::sFinalStock_[i].setZero();
			BASE::xFinalStock_[i].setZero();

			continue;
		}

		if (BASE::options_.useRiccatiSolver_==true) {
			this->solveSingleSequentialRiccatiEquation(i, learningRate, BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::sFinalStock_[i+1]);
			this->solveSingleErrorRiccatiEquation(i, BASE::SveFinalStock_[i+1]);
		} else {
			this->fullBackwardSweep(i, BASE::SmFinalStock_[i+1], BASE::SvFinalStock_[i+1], BASE::SveFinalStock_[i+1], BASE::sFinalStock_[i+1]);
		}

		// set the final value for next Riccati equation
		BASE::sFinalStock_[i]   = BASE::sTrajectoryStock_[i].front();
		BASE::SvFinalStock_[i]  = BASE::SvTrajectoryStock_[i].front();
		BASE::SveFinalStock_[i] = BASE::SveTrajectoryStock_[i].front();
		BASE::SmFinalStock_[i]  = BASE::SmTrajectoryStock_[i].front();

	}

	// state at the switching times
	BASE::xFinalStock_[BASE::finalActiveSubsystem_+1] = BASE::nominalStateTrajectoriesStock_[BASE::finalActiveSubsystem_].back();
	for (size_t i=BASE::initActiveSubsystem_; i<=BASE::finalActiveSubsystem_; i++)
		BASE::xFinalStock_[i] = BASE::nominalStateTrajectoriesStock_[i].front();

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::setupOptimizer() {

	if (BASE::subsystemDynamicsPtr_.size()-1 < *std::max_element(BASE::systemStockIndexes_.begin(), BASE::systemStockIndexes_.end()))
		throw std::runtime_error("systemStockIndex points to non-existing subsystem");

	subsystemDynamicsPtrStock_.resize(BASE::numSubsystems_);
	subsystemDerivativesPtrStock_.resize(BASE::numSubsystems_);
	subsystemCostFunctionsPtrStock_.resize(BASE::numSubsystems_);
	subsystemSimulatorsStockPtr_.resize(BASE::numSubsystems_);

	for (int i=0; i<BASE::systemStockIndexes_.size(); i++) {
		subsystemDynamicsPtrStock_[i] = BASE::subsystemDynamicsPtr_[BASE::systemStockIndexes_[i]]->clone();
		subsystemDerivativesPtrStock_[i] = BASE::subsystemDerivativesPtr_[BASE::systemStockIndexes_[i]]->clone();
		subsystemCostFunctionsPtrStock_[i] = BASE::subsystemCostFunctionsPtr_[BASE::systemStockIndexes_[i]]->clone();

		subsystemSimulatorsStockPtr_[i] =  std::shared_ptr<ODE45<STATE_DIM>>( new ODE45<STATE_DIM>(subsystemDynamicsPtrStock_[i]) );
	}

	BASE::sFinalStock_.resize(BASE::numSubsystems_+1);
	BASE::SvFinalStock_.resize(BASE::numSubsystems_+1);
	BASE::SveFinalStock_.resize(BASE::numSubsystems_+1);
	BASE::SmFinalStock_.resize(BASE::numSubsystems_+1);
	BASE::xFinalStock_.resize(BASE::numSubsystems_+1);
	for (size_t i=0; i<BASE::numSubsystems_+1; i++) {
		BASE::sFinalStock_[i].setZero();
		BASE::SvFinalStock_[i].setZero();
		BASE::SveFinalStock_[i].setZero();
		BASE::SmFinalStock_[i].setZero();
		BASE::xFinalStock_[i].setZero();
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::setSingleCostNominalState(size_t index, const scalar_array_t& timeTrajectory,
		const state_vector_array_t& stateTrajectory) {

	subsystemCostFunctionsPtrStock_[index]->setCostNominalState(timeTrajectory, stateTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::getSingleCostNominalState(size_t index, scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory) const {

	subsystemCostFunctionsPtrStock_[index]->getCostNominalState(timeTrajectory, stateTrajectory);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::runInit() {

	// set the start and final time for costFuntions
	for(size_t i=0; i<BASE::numSubsystems_; i++)
		subsystemCostFunctionsPtrStock_[i]->setTimePeriod(BASE::switchingTimes_[i], BASE::switchingTimes_[i+1]);

	// initial controller rollout
	rollout(BASE::initTime_, BASE::initState_, BASE::finalTime_, BASE::nominalControllersStock_,
			BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc1TrajectoriesStock_, BASE::EvTrajectoryStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_);

	// initial controller cost
	calculateCostFunction(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalInputTrajectoriesStock_,
			BASE::nc2TrajectoriesStock_, BASE::HvTrajectoryStock_, BASE::nc2FinalStock_, BASE::HvFinalStock_,
			BASE::nominalTotalCost_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::runIteration(const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/)  {

#ifdef BENCHMARK
	// Benchmarking
	static size_t nIterations = 0;
	static scalar_t tAvg1, tAvg2, tAvg3 = 0.0;
	nIterations++;
	auto start = std::chrono::steady_clock::now();
#endif

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

#ifdef BENCHMARK
	auto end = std::chrono::steady_clock::now();
	auto diff = end - start;
	tAvg1 = ((1.0 - 1.0/nIterations)* tAvg1) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// solve Riccati equations
	solveSequentialRiccatiEquations(1.0 /*learningRate*/, SmFinal, SvFinal, sFinal);
	// calculate controller
	if (BASE::options_.useRiccatiSolver_==true)
		calculateController();

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg2 = ((1.0 - 1.0/nIterations)* tAvg2) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
	start = std::chrono::steady_clock::now();
#endif

	// finding the optimal learningRate
	lineSearch(BASE::learningRateStar_, BASE::options_.maxLearningRateGSLQP_);

#ifdef BENCHMARK
	end = std::chrono::steady_clock::now();
	diff = end - start;
	tAvg3 = ((1.0 - 1.0/nIterations)* tAvg3) + (1.0/nIterations)*std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP<STATE_DIM, INPUT_DIM>::runExit(const state_matrix_t& SmFinal /*= state_matrix_t::Zero()*/,
		const state_vector_t& SvFinal /*= state_vector_t::Zero()*/,
		const eigen_scalar_t& sFinal /*= eigen_scalar_t::Zero()*/)  {

	// linearizing the dynamics and quadratizing the cost function along nominal trajectories
	approximateOptimalControlProblem();

	// solve Riccati equations
	solveSequentialRiccatiEquations(0.0 /*learningRate*/, SmFinal, SvFinal, sFinal);

	// calculate the nominal co-state
	this->calculateRolloutCostate(BASE::nominalTimeTrajectoriesStock_, BASE::nominalStateTrajectoriesStock_, BASE::nominalcostateTrajectoriesStock_);

}


} // namespace ocs2
