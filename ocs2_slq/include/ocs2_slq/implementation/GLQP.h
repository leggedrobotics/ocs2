/*
 * GLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */


namespace ocs2{


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::rollout(const state_vector_t& initState,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock)  {

	if (controllersStock.size() != numSubsystems_)
		throw std::runtime_error("controllersStock has less controllers then the number of subsystems");

	timeTrajectoriesStock.resize(numSubsystems_);
	stateTrajectoriesStock.resize(numSubsystems_);
	inputTrajectoriesStock.resize(numSubsystems_);

	state_vector_t x0 = initState;
	for (int i=0; i<numSubsystems_; i++) {

		timeTrajectoriesStock[i].clear();
		stateTrajectoriesStock[i].clear();

		// initialize subsystem i
		subsystemDynamicsPtrStock_[i]->initializeModel(systemStockIndexes_, switchingTimes_, x0, i, "GLQP");
		// set controller for subsystem i
		subsystemDynamicsPtrStock_[i]->setController(controllersStock[i]);
		// simulate subsystem i
		subsystemSimulatorsStockPtr_[i]->integrate(x0, switchingTimes_[i], switchingTimes_[i+1], stateTrajectoriesStock[i], timeTrajectoriesStock[i], 1e-3);

		// compute control trajectory for subsystem i
		inputTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
		for (int k=0; k<timeTrajectoriesStock[i].size(); k++)   {
			subsystemDynamicsPtrStock_[i]->computeInput(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k]);
		}

		// reset the initial state
		x0 = stateTrajectoriesStock[i].back();

		if (x0 != x0)  throw std::runtime_error("The rollout in GLQP is unstable.");
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::rolloutCost(const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

	totalCost = 0.0;
	for (int i=0; i<numSubsystems_; i++) {

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
		}

		// terminal cost
		if (i==numSubsystems_-1)  {
			scalar_t finalCost;
			subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(timeTrajectoriesStock[i].back(), stateTrajectoriesStock[i].back(), inputTrajectoriesStock[i].back());
			subsystemCostFunctionsPtrStock_[i]->terminalCost(finalCost);
			totalCost += finalCost;
		}
	}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::approximateOptimalControlProblem()  {

	AmStock_.resize(numSubsystems_);
	BmStock_.resize(numSubsystems_);
	qStock_.resize(numSubsystems_);
	QvStock_.resize(numSubsystems_);
	QmStock_.resize(numSubsystems_);
	RvStock_.resize(numSubsystems_);
	RmStock_.resize(numSubsystems_);
	PmStock_.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		subsystemDerivativesPtrStock_[i]->initializeModel(systemStockIndexes_, switchingTimes_, stateOperatingPointsStock_.at(i), i, "GLQP");
		subsystemDerivativesPtrStock_[i]->setCurrentStateAndControl(0.0, stateOperatingPointsStock_.at(i), inputOperatingPointsStock_.at(i));
		subsystemDerivativesPtrStock_[i]->getDerivativeState(AmStock_.at(i));
		subsystemDerivativesPtrStock_[i]->getDerivativesControl(BmStock_.at(i));

		subsystemCostFunctionsPtrStock_[i]->setCurrentStateAndControl(0.0, stateOperatingPointsStock_.at(i), inputOperatingPointsStock_.at(i));
		subsystemCostFunctionsPtrStock_[i]->evaluate(qStock_.at(i)(0));
		subsystemCostFunctionsPtrStock_[i]->stateDerivative(QvStock_.at(i));
		subsystemCostFunctionsPtrStock_[i]->stateSecondDerivative(QmStock_.at(i));
		subsystemCostFunctionsPtrStock_[i]->controlDerivative(RvStock_.at(i));
		subsystemCostFunctionsPtrStock_[i]->controlSecondDerivative(RmStock_.at(i));
		subsystemCostFunctionsPtrStock_[i]->stateControlDerivative(PmStock_.at(i));

		if (INFO_ON_) {
			std::cout<< "stateOperatingPoint[" << i << "]: \n" << stateOperatingPointsStock_[i].transpose() << std::endl;
			std::cout<< "inputOperatingPoint[" << i << "]: \n" << inputOperatingPointsStock_[i].transpose() << std::endl;
			std::cout<< "A[" << i << "]: \n" << AmStock_[i] << std::endl;
			std::cout<< "B[" << i << "]: \n" << BmStock_[i] << std::endl;
			std::cout<< "q[" << i << "]: \t" << qStock_[i] << std::endl;
			std::cout<< "Qv[" << i << "]: \n" << QvStock_[i].transpose() << std::endl;
			std::cout<< "Qm[" << i << "]: \n" << QmStock_[i] << std::endl;
			std::cout<< "Rv[" << i << "]: \n" << RvStock_[i].transpose() << std::endl;
			std::cout<< "Rm[" << i << "]: \n" << RmStock_[i] << std::endl;
			std::cout<< "Pm[" << i << "]: \n" << PmStock_[i] << std::endl;
		}

		// making sure that Qm is PSD
		makePSD(QmStock_[i]);

		if (i==numSubsystems_-1)  {
			subsystemCostFunctionsPtrStock_[i]->terminalCost(qFinal_(0));
			subsystemCostFunctionsPtrStock_[i]->terminalCostStateDerivative(QvFinal_);
			subsystemCostFunctionsPtrStock_[i]->terminalCostStateSecondDerivative(QmFinal_);
			// making sure that Qm is PSD
			makePSD(QmFinal_);

			if (INFO_ON_) {
				std::cout<< "qFinal[" << i << "]: \t" << qFinal_ << std::endl;
				std::cout<< "QvFinal[" << i << "]: \n" << QvFinal_.transpose() << std::endl;
				std::cout<< "QmFinal[" << i << "]: \n" << QmFinal_ << std::endl;
			}
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::calculatecontroller(const scalar_t& learningRate, controller_array_t& controllersStock) {

	for (int i=0; i<numSubsystems_; i++) {

		controllersStock[i].time_ = timeTrajectoryStock_[i];

		controllersStock[i].k_.resize(timeTrajectoryStock_[i].size());
		controllersStock[i].uff_.resize(timeTrajectoryStock_[i].size());
		for (int k=0; k<timeTrajectoryStock_[i].size(); k++) {

			control_matrix_t RmInverse = RmStock_[i].inverse();
			controllersStock[i].k_[k]    = -RmInverse * (PmStock_[i] + BmStock_[i].transpose()*SmTrajectoryStock_[i][k]);
			controllersStock[i].uff_[k]  = -learningRate * RmInverse * (RvStock_[i]  + BmStock_[i].transpose()*SvTrajectoryStock_[i][k])
								+ inputOperatingPointsStock_[i] - controllersStock[i].k_[k]*stateOperatingPointsStock_[i];
		}

		if (INFO_ON_ ) {
			std::cout << "Controller of subsystem" << i << ":" << std::endl;
			std::cout << "learningRate " << learningRate << std::endl;
			std::cout << "time: " << controllersStock[i].time_.front() << std::endl;
			std::cout << "delta_uff: " <<  (controllersStock[i].uff_[0] + controllersStock[i].k_[0]*stateOperatingPointsStock_[i]).transpose() << std::endl;
			std::cout << "u0: " <<  inputOperatingPointsStock_[i].transpose() << std::endl;
			std::cout << "k: \n" <<  controllersStock[i].k_.front() << std::endl << std::endl;
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::transformeLocalValueFuntion2Global() {

	for (int i=0; i<numSubsystems_; i++)
		for (int k=0; k<timeTrajectoryStock_[i].size(); k++) {

			sTrajectoryStock_[i][k] = sTrajectoryStock_[i][k] - stateOperatingPointsStock_[i].transpose()*SvTrajectoryStock_[i][k] +
					0.5*stateOperatingPointsStock_[i].transpose()*SmTrajectoryStock_[i][k]*stateOperatingPointsStock_[i];
			SvTrajectoryStock_[i][k] = SvTrajectoryStock_[i][k] - SmTrajectoryStock_[i][k]*stateOperatingPointsStock_[i];
		}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
bool GLQP<STATE_DIM, INPUT_DIM>::makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

	if (squareMatrix.rows() != squareMatrix.cols())  throw std::runtime_error("Not a square matrix: makePSD() method is for square matrix.");

	Eigen::SelfAdjointEigenSolver<Derived> eig(squareMatrix);
	Eigen::VectorXd lambda = eig.eigenvalues();

	bool hasNegativeEigenValue = false;
	for (size_t j=0; j<lambda.size() ; j++)
		if (lambda(j) < 0.0) {
			hasNegativeEigenValue = true;
			lambda(j) = 0.0;
		}

	if (hasNegativeEigenValue)
		squareMatrix = eig.eigenvectors() * lambda.asDiagonal() * eig.eigenvectors().inverse();
//	else
//		squareMatrix = 0.5*(squareMatrix+squareMatrix.transpose()).eval();

	return hasNegativeEigenValue;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::getController(controller_array_t& controllersStock) {
	controllersStock = controllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
		const state_vector_array2_t& stateTrajectoryStock) {

	for (size_t i = 0; i<numSubsystems_; i++)
		subsystemCostFunctionsPtrStock_[i]->setCostNominalState(timeTrajectoryStock[i], stateTrajectoryStock[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {

	int activeSubsystem = -1;
	for (int i=0; i<numSubsystems_; i++)  {
		activeSubsystem = i;
		if (switchingTimes_[i]<=time && time<switchingTimes_[i+1])
			break;
	}

	state_matrix_t Sm;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc(&timeTrajectoryStock_[activeSubsystem], &SmTrajectoryStock_[activeSubsystem]);
	SmFunc.interpolate(time, Sm);
	state_vector_t Sv;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc(&timeTrajectoryStock_[activeSubsystem], &SvTrajectoryStock_[activeSubsystem]);
	SvFunc.interpolate(time, Sv);
	eigen_scalar_t s;
	LinearInterpolation<eigen_scalar_t,Eigen::aligned_allocator<eigen_scalar_t> > sFunc(&timeTrajectoryStock_[activeSubsystem], &sTrajectoryStock_[activeSubsystem]);
	sFunc.interpolate(time, s);

	valueFuntion = (s + state.transpose()*Sv + 0.5*state.transpose()*Sm*state).eval()(0);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::SolveRiccatiEquations()  {

	timeTrajectoryStock_.resize(numSubsystems_);
	sTrajectoryStock_.resize(numSubsystems_);
	SvTrajectoryStock_.resize(numSubsystems_);
	SmTrajectoryStock_.resize(numSubsystems_);

	// final value for the last Riccati equations
	Eigen::Matrix<double,RiccatiEquations_t::S_DIM_,1> allSsFinal;
	RiccatiEquations_t::convert2Vector(QmFinal_, QvFinal_, qFinal_, allSsFinal);

	for (int i=numSubsystems_-1; i>=0; i--) {

		// set data for Riccati equations
		auto riccatiEquationsPtr = std::allocate_shared< RiccatiEquations_t,
				Eigen::aligned_allocator<RiccatiEquations_t>>(
						Eigen::aligned_allocator<RiccatiEquations_t>() );

		riccatiEquationsPtr->setData(switchingTimes_[i], switchingTimes_[i+1],
				AmStock_[i], BmStock_[i],
				qStock_[i], QvStock_[i], QmStock_[i], RvStock_[i], RmStock_[i], PmStock_[i]);

		// integrating the Riccati equations
		ODE45<RiccatiEquations_t::S_DIM_> ode45(riccatiEquationsPtr);
		scalar_array_t normalizedTimeTrajectory;
		std::vector<Eigen::Matrix<double,RiccatiEquations_t::S_DIM_,1>, Eigen::aligned_allocator<Eigen::Matrix<double,RiccatiEquations_t::S_DIM_,1>> > allSsTrajectory;
		ode45.integrate(allSsFinal, i, i+1, allSsTrajectory, normalizedTimeTrajectory);

		// denormalizing time and constructing 'Sm', 'Sv', and 's'
		int N = normalizedTimeTrajectory.size();
		timeTrajectoryStock_[i].resize(N);
		SmTrajectoryStock_[i].resize(N);
		SvTrajectoryStock_[i].resize(N);
		sTrajectoryStock_[i].resize(N);
		for (int k=0; k<N; k++) {

			RiccatiEquations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[i][k], SvTrajectoryStock_[i][k], sTrajectoryStock_[i][k]);
			timeTrajectoryStock_[i][k] = (switchingTimes_[i]-switchingTimes_[i+1])*(normalizedTimeTrajectory[N-1-k]-i) + switchingTimes_[i+1];
		}

		// testing the numerical stability of the Riccati equations
		for (int k=N-1; k>=0; k--) {
			try {
				if (SmTrajectoryStock_[i][k] != SmTrajectoryStock_[i][k])  throw std::runtime_error("Sm is unstable");
				if (SvTrajectoryStock_[i][k] != SvTrajectoryStock_[i][k])  throw std::runtime_error("Sv is unstable");
				if (sTrajectoryStock_[i][k] != sTrajectoryStock_[i][k])    throw std::runtime_error("s is unstable");
			}
			catch(std::exception const& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << timeTrajectoryStock_[i][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << timeTrajectoryStock_[i][kp] << "]:\n"<< SmTrajectoryStock_[i][kp].transpose() << std::endl;
					std::cerr << "Sv[" << timeTrajectoryStock_[i][kp] << "]:\t"<< SvTrajectoryStock_[i][kp].transpose() << std::endl;
					std::cerr << "s[" << timeTrajectoryStock_[i][kp] << "]: \t"<< sTrajectoryStock_[i][kp].transpose() << std::endl;
				}
				exit(1);
			}
		}

		// reset the final value for next Riccati equation
		allSsFinal = allSsTrajectory.back();

		if (allSsFinal != allSsFinal)
			throw std::runtime_error("Riccati Equation solver in GLQP is unstable.");

//		std::cout << "allSsFinal " << i << ":\n" << allSsFinal.transpose() << std::endl;
	}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::setupOptimizer() {

	if (subsystemDynamicsPtr_.size()-1 < *std::max_element(systemStockIndexes_.begin(), systemStockIndexes_.end()))
		throw std::runtime_error("systemStockIndexes points to non-existing subsystem");

	subsystemDynamicsPtrStock_.resize(numSubsystems_);
	subsystemDerivativesPtrStock_.resize(numSubsystems_);
	subsystemCostFunctionsPtrStock_.resize(numSubsystems_);

	stateOperatingPointsStock_.resize(numSubsystems_);
	inputOperatingPointsStock_.resize(numSubsystems_);

	subsystemSimulatorsStockPtr_.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		subsystemDynamicsPtrStock_[i] = subsystemDynamicsPtr_[systemStockIndexes_[i]]->clone();
		subsystemDerivativesPtrStock_[i] = subsystemDerivativesPtr_[systemStockIndexes_[i]]->clone();
		subsystemCostFunctionsPtrStock_[i] = subsystemCostFunctionsPtr_[systemStockIndexes_[i]]->clone();

		stateOperatingPointsStock_[i] = stateOperatingPoints_[systemStockIndexes_[i]];
		inputOperatingPointsStock_[i] = inputOperatingPoints_[systemStockIndexes_[i]];

		subsystemSimulatorsStockPtr_[i] = std::allocate_shared<
				ODE45<STATE_DIM>, Eigen::aligned_allocator<ODE45<STATE_DIM>> >(
						Eigen::aligned_allocator<ODE45<STATE_DIM>>(), subsystemDynamicsPtrStock_[i] );
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GLQP<STATE_DIM, INPUT_DIM>::run(const std::vector<size_t>& systemStockIndexes,
		const std::vector<scalar_t>& switchingTimes, const scalar_t& learningRate/*=1.0*/,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock/*=std::vector<scalar_array_t>()*/,
		const state_vector_array2_t& desiredStateTrajectoriesStock/*=state_vector_array2_t()*/)  {

	// use systemStockIndexes_ if no systemStockIndexes is given
	if (systemStockIndexes.empty()==false) {
		if (systemStockIndexes!=systemStockIndexes_) {
			numSubsystems_ = systemStockIndexes.size();
			systemStockIndexes_ = systemStockIndexes;
			setupOptimizer();
		}
	} else {
		if (systemStockIndexes_.empty()==true)
			throw std::runtime_error("systemStockIndexes should be provided since internal systemStockIndexes is empty.");
	}

	if (switchingTimes.size() != numSubsystems_+1)
		throw std::runtime_error("Number of switching times should be one plus the number of subsystems.");
	switchingTimes_ = switchingTimes;

//	// use desired Trajectories if no input is given
//	if (desiredTimeTrajectoriesStock.empty()==false) {
//		if (desiredTimeTrajectoriesStock.size() != numSubsystems_)
//			throw std::runtime_error("desiredTimeTrajectoriesStock has less controllers than the number of subsystems");
//		if (desiredStateTrajectoriesStock.size() != numSubsystems_)
//			throw std::runtime_error("desiredStateTrajectoriesStock has less controllers than the number of subsystems");
//		desiredTimeTrajectoriesStock_  = desiredTimeTrajectoriesStock;
//		desiredStateTrajectoriesStock_ = desiredStateTrajectoriesStock;
//	} else {
//		if (desiredTimeTrajectoriesStock_.empty()==true)
//			throw std::runtime_error("desiredTimeTrajectoriesStock should be provided since the intitial is empty.");
//	}
//
//	// set subsystem cost desired trajectory
//	for (size_t i=0; i<numSubsystems_; i++)
//		subsystemCostFunctionsPtrStock_[i]->setCostNominalState(desiredTimeTrajectoriesStock_[i], desiredStateTrajectoriesStock_[i]);

	// set the start and final time for costFuntions
	for(size_t i=0; i<numSubsystems_; i++)
		subsystemCostFunctionsPtrStock_[i]->setTimePeriod(switchingTimes_[i], switchingTimes_[i+1]);

	// linearizing the dynamics and quadratizing the cost funtion along nominal trajectories
	approximateOptimalControlProblem();

	// solve Riccati equations
	SolveRiccatiEquations();

	// calculate controller
	controllersStock_.resize(numSubsystems_);
	calculatecontroller(learningRate, controllersStock_);

	// transforme the local value funtion to the global representation
	transformeLocalValueFuntion2Global();
}

} // namespace ocs2

