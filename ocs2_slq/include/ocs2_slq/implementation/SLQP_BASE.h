/*
 * Implementation of SLQP_BASE.h
 *
 *  Created on: October 7, 2016
 *      Author: mgiftthaler@ethz.ch
 */


namespace ocs2
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::solveSingleSequentialRiccatiEquation(
		const size_t& index, const scalar_t& learningRate,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

//	if (index < (signed)initActiveSubsystem_   ||  index > (signed)finalActiveSubsystem_) {
//		SsTimeTrajectoryStock_[index].clear();
//		SmTrajectoryStock_[index].clear();
//		SvTrajectoryStock_[index].clear();
//		sTrajectoryStock_[index].clear();
//		SmFinalStock_[index].setZero();
//		SvFinalStock_[index].setZero();
//		sFinalStock_[index].setZero();
//		xFinalStock_[index].setZero();
//		return;
//	}

	// set data for Riccati equations
	std::shared_ptr<RiccatiEquations_t> riccatiEquationsPtr( new RiccatiEquations_t() );
	riccatiEquationsPtr->setData(learningRate, index, switchingTimes_[index], switchingTimes_[index+1],
			&nominalTimeTrajectoriesStock_[index],
			&AmConstrainedTrajectoryStock_[index], &BmTrajectoryStock_[index],
			&qTrajectoryStock_[index], &QvConstrainedTrajectoryStock_[index], &QmConstrainedTrajectoryStock_[index],
			&RvTrajectoryStock_[index], &RmInverseTrajectoryStock_[index], &RmConstrainedTrajectoryStock_[index], &PmTrajectoryStock_[index]);

	// max number of steps of integration
	size_t maxNumSteps = options_.maxNumStepsPerSecond_ * std::max( 1.0, switchingTimes_[index+1]-switchingTimes_[index] );

	// final value for the last Riccati equations plus final cost
	typename RiccatiEquations_t::s_vector_t allSsFinal;
	RiccatiEquations_t::convert2Vector(SmFinal+QmFinalStock_[index], SvFinal+QvFinalStock_[index], sFinal+qFinalStock_[index], allSsFinal);

	SsNormalizedTimeTrajectoryStock_[index].clear();
	std::vector<typename RiccatiEquations_t::s_vector_t, Eigen::aligned_allocator<typename RiccatiEquations_t::s_vector_t> > allSsTrajectory;

	// final time for reccati equation
	double finalNormalizedTime;
	if (index==initActiveSubsystem_) {
		finalNormalizedTime = (switchingTimes_[index+1]-initTime_)/(switchingTimes_[index+1]-switchingTimes_[index]);
	} else if (index==finalActiveSubsystem_) {
		finalNormalizedTime = (finalTime_-switchingTimes_[index])/(switchingTimes_[index+1]-switchingTimes_[index]);
	} else {
		finalNormalizedTime = 1.0;
	}

	switch(options_.RiccatiIntegratorType_) {

	case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ODE45 : {
		ODE45<RiccatiEquations_t::S_DIM_> riccati_integrator (riccatiEquationsPtr);
//		riccati_integrator.integrate(allSsFinal, 0.0, switchingTimes_[index+1]-switchingTimes_[index], allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index], 1e-5, options_.AbsTolODE_, options_.RelTolODE_, maxNumSteps);
		riccati_integrator.integrate(allSsFinal, 0.0, finalNormalizedTime, allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index], 1e-3, options_.AbsTolODE_, options_.RelTolODE_, maxNumSteps);
		break;
	}
	/*note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
	case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ADAMS_BASHFORTH : {
		const size_t order = 4;
		IntegratorAdamsBashforth<RiccatiEquations_t::S_DIM_,order> riccati_integrator (riccatiEquationsPtr);
//		riccati_integrator.integrate(allSsFinal, 0.0, switchingTimes_[index+1]-switchingTimes_[index], options_.adams_integrator_dt_, allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index]); // fixed time step
		riccati_integrator.integrate(allSsFinal, 0.0, finalNormalizedTime, options_.adams_integrator_dt_, allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index]); // fixed time step
		break;
	}
	case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::BULIRSCH_STOER : {
		IntegratorBulirschStoer<RiccatiEquations_t::S_DIM_> riccati_integrator (riccatiEquationsPtr);
//		riccati_integrator.integrate(allSsFinal, 0.0, switchingTimes_[index+1]-switchingTimes_[index], allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index], 1e-5, options_.AbsTolODE_, options_.RelTolODE_, maxNumSteps);
		riccati_integrator.integrate(allSsFinal, 0.0, finalNormalizedTime, allSsTrajectory, SsNormalizedTimeTrajectoryStock_[index], 1e-5, options_.AbsTolODE_, options_.RelTolODE_, maxNumSteps);
		break;
	}
	default:
		throw (std::runtime_error("Riccati equation integrator type specified wrongly in solveSequentialRiccatiEquations()"));
	}

	// denormalizing time and constructing 'Sm', 'Sv', and 's'
	int N = SsNormalizedTimeTrajectoryStock_[index].size();
	SsTimeTrajectoryStock_[index].resize(N);
	SmTrajectoryStock_[index].resize(N);
	SvTrajectoryStock_[index].resize(N);
	sTrajectoryStock_[index].resize(N);
	for (int k=0; k<N; k++) {
		RiccatiEquations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[index][k], SvTrajectoryStock_[index][k], sTrajectoryStock_[index][k]);
//		SsTimeTrajectoryStock_[index][k] = switchingTimes_[index+1] - SsNormalizedTimeTrajectoryStock_[index][N-1-k];
		SsTimeTrajectoryStock_[index][k] = (switchingTimes_[index]-switchingTimes_[index+1])*SsNormalizedTimeTrajectoryStock_[index][N-1-k] + switchingTimes_[index+1];
	}  // end of k loop

//	// set the final value for next Riccati equation
//	sFinalStock_[index]  = sTrajectoryStock_[index].front();
//	SvFinalStock_[index] = SvTrajectoryStock_[index].front();
//	SmFinalStock_[index] = SmTrajectoryStock_[index].front();

	// testing the numerical stability of the Riccati equations
	if (options_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (SmTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sv is unstable.");
				if (sTrajectoryStock_[index][k].hasNaN())   throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[index][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[index][kp] << "]:\n"<< SmTrajectoryStock_[index][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< SvTrajectoryStock_[index][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[index][kp] << "]: \t"<< sTrajectoryStock_[index][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::solveSingleSequentialRiccatiEquation(
		const size_t& index, const scalar_t& learningRate,
		const scalar_array_t& nominalTimeTrajectory,
		const state_matrix_t& SmFinal,
		const state_vector_t& SvFinal,
		const eigen_scalar_t& sFinal)  {

//	if (index < (signed)initActiveSubsystem_   ||  index > (signed)finalActiveSubsystem_) {
//		SsTimeTrajectoryStock_[index].clear();
//		SmTrajectoryStock_[index].clear();
//		SvTrajectoryStock_[index].clear();
//		sTrajectoryStock_[index].clear();
//		SmFinalStock_[index].setZero();
//		SvFinalStock_[index].setZero();
//		sFinalStock_[index].setZero();
//		xFinalStock_[index].setZero();
//		return;
//	}

	// set data for Riccati equations
	std::shared_ptr<RiccatiEquations_t> riccatiEquationsPtr( new RiccatiEquations_t() );
	riccatiEquationsPtr->setData(learningRate, index, switchingTimes_[index], switchingTimes_[index+1],
			&nominalTimeTrajectory,
			&AmConstrainedTrajectoryStock_[index], &BmTrajectoryStock_[index],
			&qTrajectoryStock_[index], &QvConstrainedTrajectoryStock_[index], &QmConstrainedTrajectoryStock_[index],
			&RvTrajectoryStock_[index], &RmInverseTrajectoryStock_[index], &RmConstrainedTrajectoryStock_[index], &PmTrajectoryStock_[index]);

	// max number of steps of integration
	size_t maxNumSteps = options_.maxNumStepsPerSecond_ * std::max( 1.0, switchingTimes_[index+1]-switchingTimes_[index] );

	// final value for the last Riccati equations plus final cost
	typename RiccatiEquations_t::s_vector_t allSsFinal;
	RiccatiEquations_t::convert2Vector(SmFinal+QmFinalStock_[index], SvFinal+QvFinalStock_[index], sFinal+qFinalStock_[index], allSsFinal);

	size_t N = nominalTimeTrajectory.size();
	SsNormalizedTimeTrajectoryStock_[index].resize(N);
	for (int k=0; k<N; k++)
		SsNormalizedTimeTrajectoryStock_[index][N-1-k] = (nominalTimeTrajectory[k] - switchingTimes_[index+1]) / (switchingTimes_[index]-switchingTimes_[index+1]);

	std::vector<typename RiccatiEquations_t::s_vector_t, Eigen::aligned_allocator<typename RiccatiEquations_t::s_vector_t> > allSsTrajectory;

	ODE45<RiccatiEquations_t::S_DIM_> riccati_integrator (riccatiEquationsPtr);
	riccati_integrator.integrate(allSsFinal, SsNormalizedTimeTrajectoryStock_[index], allSsTrajectory, 1e-5, options_.AbsTolODE_, options_.RelTolODE_);

	// denormalizing time and constructing 'Sm', 'Sv', and 's'
	SsTimeTrajectoryStock_[index] = nominalTimeTrajectory;
	SmTrajectoryStock_[index].resize(N);
	SvTrajectoryStock_[index].resize(N);
	sTrajectoryStock_[index].resize(N);
	for (int k=0; k<N; k++) {
		RiccatiEquations_t::convert2Matrix(allSsTrajectory[N-1-k], SmTrajectoryStock_[index][k], SvTrajectoryStock_[index][k], sTrajectoryStock_[index][k]);
	}  // end of k loop

//	// set the final value for next Riccati equation
//	sFinalStock_[index]  = sTrajectoryStock_[index].front();
//	SvFinalStock_[index] = SvTrajectoryStock_[index].front();
//	SmFinalStock_[index] = SmTrajectoryStock_[index].front();

	// testing the numerical stability of the Riccati equations
	if (options_.checkNumericalStability_)
		for (int k=N-1; k>=0; k--) {
			try {
				if (SmTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sv is unstable.");
				if (sTrajectoryStock_[index][k].hasNaN())   throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error)
			{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[index][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[index][kp] << "]:\n"<< SmTrajectoryStock_[index][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< SvTrajectoryStock_[index][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[index][kp] << "]: \t"<< sTrajectoryStock_[index][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::solveSingleErrorRiccatiEquation(const size_t& index,
		const state_vector_t& SveFinal)  {
	/*
	 * Type_1 constraints error correction compensation
	 */

//	if (index < (signed)initActiveSubsystem_  ||  index > (signed)finalActiveSubsystem_) {
//		SveTrajectoryStock_[index].clear();
//		SveFinalStock_[index].setZero();
//		return;
//	}

	int N = SsNormalizedTimeTrajectoryStock_[index].size();

	// Skip calculation of the error correction term Sve if the constrained simulation is used for forward simulation
	if (options_.simulationIsConstrained_) {
		SveTrajectoryStock_[index].resize(N);
		for (int k=0; k<N; k++)
			SveTrajectoryStock_[index][k].setZero();
		return;
	}

	LinearInterpolation<state_matrix_t, Eigen::aligned_allocator<state_matrix_t> > SmFunc;

	// Calculating the coefficients of the error equation
	SmFunc.setTimeStamp( &(SsTimeTrajectoryStock_[index]) );
	SmFunc.setData( &(SmTrajectoryStock_[index]) );

	state_vector_array_t GvTrajectory(nominalTimeTrajectoriesStock_[index].size());
	state_matrix_array_t  GmTrajectory(nominalTimeTrajectoriesStock_[index].size());

	for (int k=nominalTimeTrajectoriesStock_[index].size()-1; k>=0; k--) {

		state_matrix_t Sm;
		SmFunc.interpolate(nominalTimeTrajectoriesStock_[index][k], Sm);

		control_feedback_t Lm = RmInverseTrajectoryStock_[index][k]*(PmTrajectoryStock_[index][k]+BmTrajectoryStock_[index][k].transpose()*Sm);

		GmTrajectory[k] = AmConstrainedTrajectoryStock_[index][k] -
				BmTrajectoryStock_[index][k]*RmInverseTrajectoryStock_[index][k]*RmConstrainedTrajectoryStock_[index][k]*Lm;

		GvTrajectory[k] = (CmProjectedTrajectoryStock_[index][k]-Lm).transpose()*
				RmTrajectoryStock_[index][k]*EvProjectedTrajectoryStock_[index][k];

	}  // end of k loop

	// set data for error equations
	std::shared_ptr<ErrorEquation_t> errorEquationPtr( new ErrorEquation_t() );
	errorEquationPtr->setData(index, switchingTimes_[index], switchingTimes_[index+1], &nominalTimeTrajectoriesStock_[index], &GvTrajectory, &GmTrajectory);

	// integrating the Riccati equations
	ODE45<STATE_DIM> errorOde45(errorEquationPtr);
	state_vector_array_t SveTrajectory;
//	errorOde45.integrate(SveFinal, SsNormalizedTimeTrajectoryStock_[index], SveTrajectory, 1e-3, options_.AbsTolODE_, options_.RelTolODE_);
	errorOde45.integrate(SveFinal, SsNormalizedTimeTrajectoryStock_[index], SveTrajectory, 1e-3, options_.AbsTolODE_, options_.RelTolODE_);

//	// set the final value for next Riccati equation
//	SveFinalStock_[index] = SveTrajectory.back();

	// constructing ''Sve' and testing the numerical stability
	SveTrajectoryStock_[index].resize(N);
	for (int k=0; k<N; k++) {
		SveTrajectoryStock_[index][k] = SveTrajectory[N-1-k];

		if (options_.checkNumericalStability_){
			// testing the numerical stability of the Riccati error equation
			try {
				if (SveTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sve is unstable");
			}
			catch(const std::exception& error) 	{
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[index][k] << " [sec]." << std::endl;
				for (int kp=k; kp<N; kp++){
					std::cerr << "Sve[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< SveTrajectoryStock_[index][kp].transpose().norm() << std::endl;
				}
				for(int kp = 0; kp+1<nominalTimeTrajectoriesStock_[index].size(); kp++){
					std::cerr << "Gm[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< GmTrajectory[kp].transpose().norm() << std::endl;
					std::cerr << "Gv[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< GvTrajectory[kp].transpose().norm() << std::endl;
				}
				exit(0);
			}
		}
	}  // end of k loop

	if(SveTrajectory.size()!=N)  throw std::runtime_error("sve traj size not equal to N");

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template<size_t DIM1, size_t DIM2>
Eigen::Matrix<double, DIM1, DIM2> SLQP_BASE<STATE_DIM, INPUT_DIM>::solveLTIMatrix(
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
template <size_t STATE_DIM, size_t INPUT_DIM>
template<int DIM1>
Eigen::Matrix<double, DIM1, 1> SLQP_BASE<STATE_DIM, INPUT_DIM>::solveLTI(
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

	scalar_array_t timeTrajectory(2);
	timeTrajectory[0] = 0.0;
	timeTrajectory[1] = deltaTime;
	std::vector<Eigen::Matrix<double, DIM1, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, DIM1, 1>> > stateTrajectory;
	firstOrderOde45.integrate(x0, timeTrajectory,
			stateTrajectory,
			0.01, options_.AbsTolODE_, options_.RelTolODE_);

	Eigen::Matrix<double, DIM1, 1> xf = stateTrajectory.back();

	return xf;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::fullBackwardSweep(const size_t& index,
		const state_matrix_t& SmFinal, const state_vector_t& SvFinal,
		const state_vector_t& SveFinal, const eigen_scalar_t& sFinal)  {

//	if (index < (signed)initActiveSubsystem_ || index > (signed)finalActiveSubsystem_) {
//		SsTimeTrajectoryStock_[index].clear();
//		SmTrajectoryStock_[index].clear();
//		SvTrajectoryStock_[index].clear();
//		SveTrajectoryStock_[index].clear();
//		sTrajectoryStock_[index].clear();
//		SmFinalStock_[index].setZero();
//		SvFinalStock_[index].setZero();
//		SveFinalStock_[index].setZero();
//		sFinalStock_[index].setZero();
//		xFinalStock_[index].setZero();
//		nominalControllersStock_[index].clear();
//		return;
//	}

	size_t N = nominalTimeTrajectoriesStock_[index].size();

	state_matrix_t _Gm;
	state_vector_t _Gv, _Gve;
	control_feedback_t _Lm, _LmConstrained;
	control_vector_t _LvConstrained, _LveConstrained;
	Eigen::Matrix<scalar_t, 2*STATE_DIM, STATE_DIM> _X_H_0, _X_H_1;
	Eigen::Matrix<scalar_t, 2*STATE_DIM, 2*STATE_DIM> _H;
	_X_H_0.template topRows<STATE_DIM>() = state_matrix_t::Identity();

	// Riccati parameters
	SmTrajectoryStock_[index].resize(N);
	SvTrajectoryStock_[index].resize(N);
	SveTrajectoryStock_[index].resize(N);
	sTrajectoryStock_[index].resize(N);

	SsTimeTrajectoryStock_[index]   = nominalTimeTrajectoriesStock_[index];
	SmTrajectoryStock_[index][N-1]  = SmFinal + QmFinalStock_[index];
	SvTrajectoryStock_[index][N-1]  = SvFinal + QvFinalStock_[index];
	SveTrajectoryStock_[index][N-1] = SveFinal;
	sTrajectoryStock_[index][N-1]   = sFinal + qFinalStock_[index];

	// controller parameters
	nominalControllersStock_[index].time_ = nominalTimeTrajectoriesStock_[index];
	nominalControllersStock_[index].k_.resize(N);
	nominalControllersStock_[index].uff_.resize(N);
	nominalControllersStock_[index].deltaUff_.resize(N);

	_Lm 		   = -RmInverseTrajectoryStock_[index][N-1] * (
			PmTrajectoryStock_[index][N-1] + BmTrajectoryStock_[index][N-1].transpose()*SmTrajectoryStock_[index][N-1]);
	_LmConstrained = (control_matrix_t::Identity() - DmProjectedTrajectoryStock_[index][N-1]) * _Lm;
	_LvConstrained = -RmInverseTrajectoryStock_[index][N-1] *
			(RvConstrainedTrajectoryStock_[index][N-1] + BmConstrainedTrajectoryStock_[index][N-1].transpose()*SvTrajectoryStock_[index][N-1]);
	_LveConstrained = -RmInverseTrajectoryStock_[index][N-1] * BmConstrainedTrajectoryStock_[index][N-1].transpose() * SveTrajectoryStock_[index][N-1];
	nominalControllersStock_[index].k_[N-1]   = _LmConstrained - CmProjectedTrajectoryStock_[index][N-1];
	nominalControllersStock_[index].uff_[N-1] = nominalInputTrajectoriesStock_[index][N-1]
			- nominalControllersStock_[index].k_[N-1]*nominalStateTrajectoriesStock_[index][N-1]
			+ options_.constraintStepSize_ * (_LveConstrained - EvProjectedTrajectoryStock_[index][N-1]);
	nominalControllersStock_[index].deltaUff_[N-1] = _LvConstrained;

	for (int k=N-2; k>=0; k--) {

		double deltaT = nominalTimeTrajectoriesStock_[index][k] - nominalTimeTrajectoriesStock_[index][k+1];

		makePSD(QmConstrainedTrajectoryStock_[index][k]);
		_H.template topLeftCorner<STATE_DIM,STATE_DIM>() = AmConstrainedTrajectoryStock_[index][k] -
				BmConstrainedTrajectoryStock_[index][k]*RmInverseTrajectoryStock_[index][k]*PmConstrainedTrajectoryStock_[index][k];
		_H.template topRightCorner<STATE_DIM,STATE_DIM>() = 0.5 * BmConstrainedTrajectoryStock_[index][k] *
				RmInverseTrajectoryStock_[index][k] * BmConstrainedTrajectoryStock_[index][k].transpose();
		_H.template bottomLeftCorner<STATE_DIM,STATE_DIM>() = 2.0 * (QmConstrainedTrajectoryStock_[index][k] -
				PmConstrainedTrajectoryStock_[index][k].transpose()*RmInverseTrajectoryStock_[index][k]*PmConstrainedTrajectoryStock_[index][k]);
		_H.template bottomRightCorner<STATE_DIM,STATE_DIM>() = -_H.template topLeftCorner<STATE_DIM,STATE_DIM>().transpose();

		_X_H_0.template bottomRows<STATE_DIM>() = -2.0*SmTrajectoryStock_[index][k+1];
		_X_H_1 = solveLTIMatrix<2*STATE_DIM, STATE_DIM>(_H, _X_H_0, deltaT);
		SmTrajectoryStock_[index][k] = -0.5 * _X_H_1.template bottomRows<STATE_DIM>() * _X_H_1.template topRows<STATE_DIM>().inverse();

		_Lm 		   = -RmInverseTrajectoryStock_[index][k] * (PmTrajectoryStock_[index][k] + BmTrajectoryStock_[index][k].transpose()*SmTrajectoryStock_[index][k]);
		_LmConstrained = (control_matrix_t::Identity() - DmProjectedTrajectoryStock_[index][k]) * _Lm;

		_Gm  = (AmConstrainedTrajectoryStock_[index][k] + BmConstrainedTrajectoryStock_[index][k]*_LmConstrained).transpose();
		_Gv  = (QvConstrainedTrajectoryStock_[index][k] + _LmConstrained.transpose()*RvConstrainedTrajectoryStock_[index][k]);
		_Gve = (CmProjectedTrajectoryStock_[index][k] + _Lm).transpose() * RmTrajectoryStock_[index][k] * EvProjectedTrajectoryStock_[index][k];

		SvTrajectoryStock_[index][k]  = solveLTI<STATE_DIM>(_Gm, _Gv,  SvTrajectoryStock_[index][k+1],  -deltaT);
		SveTrajectoryStock_[index][k] = solveLTI<STATE_DIM>(_Gm, _Gve, SveTrajectoryStock_[index][k+1], -deltaT);

		_LvConstrained  = -RmInverseTrajectoryStock_[index][k] *
				(RvConstrainedTrajectoryStock_[index][k] + BmConstrainedTrajectoryStock_[index][k].transpose()*SvTrajectoryStock_[index][k]);
		_LveConstrained = -RmInverseTrajectoryStock_[index][k] * BmConstrainedTrajectoryStock_[index][k].transpose() * SveTrajectoryStock_[index][k];

		sTrajectoryStock_[index][k] = sTrajectoryStock_[index][k+1] - deltaT * qTrajectoryStock_[index][k];

		// controller
		nominalControllersStock_[index].k_[k]   = _LmConstrained - CmProjectedTrajectoryStock_[index][k];
		nominalControllersStock_[index].uff_[k] = nominalInputTrajectoriesStock_[index][k]
		        - nominalControllersStock_[index].k_[k]*nominalStateTrajectoriesStock_[index][k]
				+ options_.constraintStepSize_ * (_LveConstrained - EvProjectedTrajectoryStock_[index][k]);
		nominalControllersStock_[index].deltaUff_[k] = _LvConstrained;
	}

//	// set the final value for next Riccati equation
//	SmFinalStock_[index]  = SmTrajectoryStock_[index].front();
//	SvFinalStock_[index]  = SvTrajectoryStock_[index].front();
//	SveFinalStock_[index] = SveTrajectoryStock_[index].front();
//	sFinalStock_[index]   = sTrajectoryStock_[index].front();


	// testing the numerical stability
	if (options_.checkNumericalStability_==true)
		for (int k=N-1; k>=0; k--) {
			// checking the numerical stability of the Riccati equations
			try {
				if (SmTrajectoryStock_[index][k].hasNaN())   throw std::runtime_error("Sm is unstable.");
				if (SvTrajectoryStock_[index][k].hasNaN())   throw std::runtime_error("Sv is unstable.");
				if (SveTrajectoryStock_[index][k].hasNaN())  throw std::runtime_error("Sve is unstable.");
				if (sTrajectoryStock_[index][k].hasNaN())    throw std::runtime_error("s is unstable.");
			}
			catch(const std::exception& error) {
				std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoryStock_[index][k] << " [sec]." << std::endl;
				for (int kp=k; kp<k+10; kp++)  {
					if (kp >= N) continue;
					std::cerr << "Sm[" << SsTimeTrajectoryStock_[index][kp] << "]:\n"<< SmTrajectoryStock_[index][kp].norm() << std::endl;
					std::cerr << "Sv[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< SvTrajectoryStock_[index][kp].transpose().norm() << std::endl;
					std::cerr << "Sve[" << SsTimeTrajectoryStock_[index][kp] << "]:\t"<< SveTrajectoryStock_[index][kp].transpose().norm() << std::endl;
					std::cerr << "s["  << SsTimeTrajectoryStock_[index][kp] << "]: \t"<< sTrajectoryStock_[index][kp].transpose().norm() << std::endl;
				}
				exit(0);
			}

			// checking the numerical stability of the controller parameters
			try {
				if (nominalControllersStock_[index].k_[k].hasNaN())        throw std::runtime_error("Feedback gains are unstable.");
				if (nominalControllersStock_[index].uff_[k].hasNaN())      throw std::runtime_error("uff gains are unstable.");
				if (nominalControllersStock_[index].deltaUff_[k].hasNaN()) throw std::runtime_error("deltaUff is unstable.");
			}
			catch(const std::exception& error) {
				std::cerr << "what(): " << error.what() << " at time " << nominalControllersStock_[index].time_[k] << " [sec]." << std::endl;
				exit(0);
			}
		}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
template <typename Derived>
bool SLQP_BASE<STATE_DIM, INPUT_DIM>::makePSD(Eigen::MatrixBase<Derived>& squareMatrix) {

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutLagrangeMultiplier(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock,
		std::vector<std::vector<Eigen::VectorXd>>& lagrangeTrajectoriesStock)  {

	typedef Eigen::Matrix<double, Eigen::Dynamic, 1> constraint_vector_t;
	typedef Eigen::Matrix<double, Eigen::Dynamic, STATE_DIM> constraint_matrix_t;

	lagrangeTrajectoriesStock.resize(numSubsystems_);

	LinearInterpolation<constraint_vector_t, Eigen::aligned_allocator<constraint_vector_t> > vffFunc;
	LinearInterpolation<constraint_matrix_t, Eigen::aligned_allocator<constraint_matrix_t> > vfbFunc;

	for (int i=0; i<numSubsystems_; i++) {

		size_t N = timeTrajectoriesStock[i].size();

		// if the subsystem is not simulated (e.g. due to the initial time)
		if (N==0) {
			lagrangeTrajectoriesStock[i].clear();
			continue;
		}

		vffFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vffFunc.setData(&lagrangeMultiplierFunctionsStock[i].uff_);

		vfbFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vfbFunc.setData(&lagrangeMultiplierFunctionsStock[i].k_);

		lagrangeTrajectoriesStock[i].resize(N);

		for (int k=0; k<N; k++) {

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::calculateRolloutCostate(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		state_vector_array2_t& costateTrajectoriesStock)  {


	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > SmFunc;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > SvFunc;
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > nominalStateFunc;

	costateTrajectoriesStock.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		size_t N = timeTrajectoriesStock[i].size();

		// if the subsystem is not simulated (e.g. due to the initial time)
		if (N==0) {
			costateTrajectoriesStock[i].clear();
			continue;
		}

		SmFunc.setTimeStamp(&SsTimeTrajectoryStock_[i]);
		SmFunc.setData(&SmTrajectoryStock_[i]);
		SvFunc.setTimeStamp(&SsTimeTrajectoryStock_[i]);
		SvFunc.setData(&SvTrajectoryStock_[i]);
		nominalStateFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		nominalStateFunc.setData(&nominalStateTrajectoriesStock_[i]);

		costateTrajectoriesStock[i].resize(N);

		for (int k=0; k<N; k++) {

			const double& t = timeTrajectoriesStock[i][k];

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::calculateInputConstraintLagrangian(std::vector<lagrange_t>& lagrangeMultiplierFunctionsStock) {

	// functions for controller and lagrane multiplier
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> >     nominalStateFunc;

	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmInverseFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     RvFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> >     EvProjectedFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > CmProjectedFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmFunc;
	LinearInterpolation<control_constraint1_matrix_t,Eigen::aligned_allocator<control_constraint1_matrix_t> > DmDagerFunc;

	lagrangeMultiplierFunctionsStock.resize(numSubsystems_);

	for (size_t i=0; i<numSubsystems_; i++) {

		if (i<initActiveSubsystem_ || i>finalActiveSubsystem_) {
			lagrangeMultiplierFunctionsStock[i].clear();
			continue;
		}

		nominalStateFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc.setData( &(nominalStateTrajectoriesStock_[i]) );

		BmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		BmFunc.setData( &(BmTrajectoryStock_[i]) );

		PmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(PmTrajectoryStock_[i]) );

		RmInverseFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RmInverseFunc.setData( &(RmInverseTrajectoryStock_[i]) );

		RvFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RvFunc.setData( &(RvTrajectoryStock_[i]) );

		EvProjectedFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		EvProjectedFunc.setData( &(EvProjectedTrajectoryStock_[i]) );

		CmProjectedFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		CmProjectedFunc.setData( &(CmProjectedTrajectoryStock_[i]) );

		RmFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		RmFunc.setData( &(RmTrajectoryStock_[i]) );

		DmDagerFunc.setTimeStamp( &(nominalTimeTrajectoriesStock_[i]) );
		DmDagerFunc.setData( &(DmDagerTrajectoryStock_[i]) );

		size_t N = SsTimeTrajectoryStock_[i].size();

		lagrangeMultiplierFunctionsStock[i].time_ = SsTimeTrajectoryStock_[i];
		lagrangeMultiplierFunctionsStock[i].k_.resize(N);
		lagrangeMultiplierFunctionsStock[i].uff_.resize(N);
		lagrangeMultiplierFunctionsStock[i].deltaUff_.resize(N);

		for (int k=0; k<N; k++) {

			const double& time = SsTimeTrajectoryStock_[i][k];
			size_t greatestLessTimeStampIndex;

			state_vector_t nominalState;
			nominalStateFunc.interpolate(time, nominalState);
			greatestLessTimeStampIndex = nominalStateFunc.getGreatestLessTimeStampIndex();

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

			control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*SmTrajectoryStock_[i][k]);
			control_vector_t   Lv  = RmInverse * (Rv + Bm.transpose()*SvTrajectoryStock_[i][k]);
			control_vector_t   Lve = RmInverse * (Bm.transpose()*SveTrajectoryStock_[i][k]);

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::calculateMeritFunction(
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
	if (options_.maxIterationGSLQP_>1)
		pho = iteration_/(options_.maxIterationGSLQP_-1) * options_.meritFunctionRho_;

	meritFunctionValue += 0.5*pho*constraintISE;

	// add the the lagrangian term for the constraint
	scalar_t currentIntermediateMerit;
	scalar_t nextIntermediateMerit;

	for (int i=0; i<numSubsystems_; i++)
	{
		// integrates the intermediate merit using the trapezoidal approximation method
		currentIntermediateMerit = 0.0;
		nextIntermediateMerit = 0.0;
		for (int k=0; k+1<timeTrajectoriesStock[i].size(); k++)
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
template <size_t STATE_DIM, size_t INPUT_DIM>
double SLQP_BASE<STATE_DIM, INPUT_DIM>::calculateConstraintISE(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoriesStock,
		scalar_t& constraintISE)  {

	constraintISE = 0.0;
	double maxConstraintNorm = 0.0;

	scalar_t currentSquaredNormError;
	scalar_t nextSquaredNormError;

	for (size_t i=0; i<numSubsystems_; i++)  {

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getNominalTrajectories (std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
		state_vector_array2_t& nominalStateTrajectoriesStock,
		control_vector_array2_t& nominalInputTrajectoriesStock)   {

	nominalTimeTrajectoriesStock   = nominalTimeTrajectoriesStock_;
	nominalStateTrajectoriesStock  = nominalStateTrajectoriesStock_;
	nominalInputTrajectoriesStock  = nominalInputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getNominalTrajectoriesPtr(std::shared_ptr<std::vector<scalar_array_t>>& nominalTimeTrajectoriesStock,
		std::shared_ptr<state_vector_array2_t>& nominalStateTrajectoriesStock,
		std::shared_ptr<control_vector_array2_t>& nominalInputTrajectoriesStock)   {

	nominalTimeTrajectoriesStock   = std::make_shared<std::vector<scalar_array_t>>(nominalTimeTrajectoriesStock_);
	nominalStateTrajectoriesStock  = std::make_shared<state_vector_array2_t>(nominalStateTrajectoriesStock_);
	nominalInputTrajectoriesStock  = std::make_shared<control_vector_array2_t>(nominalInputTrajectoriesStock_);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
size_t SLQP_BASE<STATE_DIM, INPUT_DIM>::findActiveSubsystemIndex(
		const scalar_array_t& switchingTimes,
		const double& time) {

	const size_t numSubsystems = switchingTimes.size();
	size_t activeSubsystemIndex = numSubsystems;  // a non-existing index
	double timePlus  = time + 1e-6; //std::numeric_limits<double>::epsilon();
	double timeMinus = time - 1e-6; //std::numeric_limits<double>::epsilon();
	for (size_t i=0; i<numSubsystems; i++)
		if (switchingTimes[i]<=timePlus && timeMinus<switchingTimes[i+1]) {
			activeSubsystemIndex = i;
			break;
		}

//	std::cout << "time: " << time << " activeSubsystemIndex: " << activeSubsystemIndex <<std::endl;

//	if (std::fabs(time-switchingTimes.back()) < 1e-6)
//		activeSubsystemIndex = numSubsystems_-1;

	if (activeSubsystemIndex == numSubsystems) {
		std::string mesg = "Given time is greater than the final time (i.e. switchingTimes.back() < givenTime): "
				+ std::to_string(switchingTimes.back()) + " < " + std::to_string(time);
		throw std::runtime_error(mesg);
	}

	return activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getValueFuntion (
		const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {

	size_t activeSubsystem = findActiveSubsystemIndex(switchingTimes_, time);

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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getCostFuntion(
		scalar_t& costFunction, scalar_t& constriantISE)  {

	costFunction = nominalTotalCost_;
	constriantISE = nominalConstraint1ISE_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getController(controller_array_t& controllersStock) const {

	controllersStock = nominalControllersStock_;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getControllerPtr(std::shared_ptr<controller_array_t>& controllersStock) const {

	controllersStock = std::make_shared<controller_array_t>(nominalControllersStock_);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
const typename SLQP_BASE<STATE_DIM, INPUT_DIM>::controller_t& SLQP_BASE<STATE_DIM, INPUT_DIM>::controller(size_t index) const {

	return nominalControllersStock_[index];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::truncateConterller(
		const scalar_array_t& switchingTimes,
		const double& initTime,
		controller_array_t& controllersStock,
		size_t& initActiveSubsystemIndex,
		controller_array_t& deletedcontrollersStock) {

	deletedcontrollersStock.resize(numSubsystems_);
	for (size_t i=0; i<numSubsystems_; i++)
		deletedcontrollersStock[i].clear();

	// finding the active subsystem index at initTime_
	initActiveSubsystemIndex = findActiveSubsystemIndex(switchingTimes, initTime);

	// saving the deleting part and clearing controllersStock
	for (size_t i=0; i<initActiveSubsystemIndex; i++)
		deletedcontrollersStock[i].swap(controllersStock[i]);

	if (controllersStock[initActiveSubsystemIndex].time_.empty()==true)  return;

	// interpolating uff
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > uffFunc;
	uffFunc.setTimeStamp(&controllersStock[initActiveSubsystemIndex].time_);
	uffFunc.setData(&controllersStock[initActiveSubsystemIndex].uff_);
	control_vector_t uffInit;
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
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::getCostNominalStates(std::vector<scalar_array_t>& timeTrajectoryStock,
		state_vector_array2_t& stateTrajectoryStock) const {
	for (size_t i=0; i<numSubsystems_; i++)
		getSingleCostNominalState(i, timeTrajectoryStock[i], stateTrajectoryStock[i]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::setCostNominalStates(const std::vector<scalar_array_t>& timeTrajectoryStock,
		const state_vector_array2_t& stateTrajectoryStock) {

	for (size_t i=initActiveSubsystem_; i<=finalActiveSubsystem_; i++)
		setSingleCostNominalState(i, timeTrajectoryStock[i], stateTrajectoryStock[i]);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::rewindOptimizer(const size_t& firstIndex, bool initRun/*=false*/) {

	if (initRun==true) {
		sFinalStock_.resize(numSubsystems_+1);
		SvFinalStock_.resize(numSubsystems_+1);
		SveFinalStock_.resize(numSubsystems_+1);
		SmFinalStock_.resize(numSubsystems_+1);
		xFinalStock_.resize(numSubsystems_+1);
		nominalControllersStock_.resize(numSubsystems_);
//		for (size_t i=0; i<numSubsystems_+1; i++) {
//			sFinalStock_[i].setZero();
//			SvFinalStock_[i].setZero();
//			SveFinalStock_[i].setZero();
//			SmFinalStock_[i].setZero();
//			xFinalStock_[i].setZero();
//		}
	}

	if (firstIndex > numSubsystems_)
		throw std::runtime_error("Index for rewind is greater than the current size");

	const size_t preservedLength = numSubsystems_ + 1 - firstIndex;
	for (size_t i=0; i<numSubsystems_+1; i++)
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

	for (size_t i=0; i<numSubsystems_; i++)
		if (i<preservedLength-1)
			nominalControllersStock_[i].swap(nominalControllersStock_[firstIndex+i]);
		else
			nominalControllersStock_[i].clear();

	isRewinded_ = true;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void SLQP_BASE<STATE_DIM, INPUT_DIM>::run(
		const double& initTime, const state_vector_t& initState, const double& finalTime,
		const std::vector<size_t>& systemStockIndexes/*=std::vector<size_t>()*/,
		const std::vector<scalar_t>& switchingTimes/*=std::vector<scalar_t>()*/,
		const controller_array_t& controllersStock/*=controller_array_t()*/,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock/*=scalar_array2_t()*/,
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

	// use switchingTimes if no switchingTimes is given
	if (switchingTimes.empty()==false) {
		if (switchingTimes.size() != numSubsystems_+1)
			throw std::runtime_error("Number of switching times should be one plus the number of subsystems.");
		switchingTimes_ = switchingTimes;
	} else {
		if (switchingTimes_.size() != numSubsystems_+1)
			throw std::runtime_error("switchingTimes should be provided since internal one is not compatible with systemStockIndexes.");
	}

	// use nominalControllersStock_ if no initial controller is given
	if (controllersStock.empty()==false) {
		if (controllersStock.size() != numSubsystems_)
			throw std::runtime_error("controllersStock has less controllers than the number of subsystems");
		nominalControllersStock_ = controllersStock;
	} else {
		if (nominalControllersStock_.empty()==true) {
			nominalControllersStock_.resize(numSubsystems_);
			std::cerr << "WARNING: Initial controllersStock is not provided. SLQ uses internal LQP controller." << std::endl;
		}
		// do nothing and use the internal controller as the initial controller (nominalControllersStock_)
	}

	// use desired Trajectories if no input is given
	if (desiredTimeTrajectoriesStock.empty()==false) {
		if (desiredTimeTrajectoriesStock.size() != numSubsystems_)
			throw std::runtime_error("desiredTimeTrajectoriesStock has less elements than the number of subsystems");
		if (desiredStateTrajectoriesStock.size() != numSubsystems_)
			throw std::runtime_error("desiredStateTrajectoriesStock has less elements than the number of subsystems");
		desiredTimeTrajectoriesStock_  = desiredTimeTrajectoriesStock;
		desiredStateTrajectoriesStock_ = desiredStateTrajectoriesStock;
	} else {
		if (desiredTimeTrajectoriesStock_.empty()==true)
			throw std::runtime_error("desiredTimeTrajectoriesStock should be provided since the intitial is empty.");
	}

	// display
	if (options_.dispayGSLQP_) {
		std::cerr << "\n#### SLQP solver starts from initial time " << initTime << " to final time " << finalTime;
		std::cerr << " with switching times \n[" << switchingTimes_[0];
		for (size_t i=1; i<=numSubsystems_; i++)   std::cerr << ", " << switchingTimes_[i];
		std::cerr << "]" << std::endl << std::endl;
	}

	initState_ = initState;
	initTime_  = initTime;
	finalTime_ = finalTime;
	iteration_ = 0;

	// finding the active subsystem index at initTime_ and truncating the controller
	truncateConterller(switchingTimes_, initTime_, nominalControllersStock_, initActiveSubsystem_, deletedcontrollersStock_);

	// final active subsystem
	finalActiveSubsystem_ = findActiveSubsystemIndex(switchingTimes_, finalTime_);

	// set desired trajectories to subsystem cost function
	setCostNominalStates(desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);

	// run loop initializer and update the member variables
	runInit();

	// initial controller constraint type-1 ISE
	calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nominalConstraint1ISE_);

	iterationCost_.clear();
	iterationISE1_.clear();
	iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
	iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );

	// display
	if (options_.dispayGSLQP_)
		std::cerr << "\n#### Initial controller: \n cost: " << nominalTotalCost_ << " \t constraint ISE: " << nominalConstraint1ISE_ << std::endl;

	// convergence conditions varaibles
	double relCost;
	double relConstraint1ISE;
	bool isConstraint1Satisfied  = false;
	bool isCostFunctionConverged = false;
	bool isOptimizationConverged = false;

//	// start profiling
//	std::string profilingFile = "/home/farbod/Programs/slq_ws/src/c_hyq_optimization/config/trotting/profiling_results_new.txt";
//	ProfilerStart(profilingFile.c_str());

	// SLQP main loop
	while (iteration_<options_.maxIterationGSLQP_ && isOptimizationConverged==false)  {

		double costCashed = nominalTotalCost_;
		double constraint1ISECashed = nominalConstraint1ISE_;

		// display
		if (options_.dispayGSLQP_)
			std::cerr << "\n#### Iteration " <<  iteration_+1 << std::endl;

		// run the an itration of the SLQ algorithm and update the member variables
		runIteration();

		// calculates type-1 constraint ISE and maximum norm
		constraint1MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc1TrajectoriesStock_, EvTrajectoryStock_, nominalConstraint1ISE_);

		iterationCost_.push_back( (Eigen::VectorXd(1) << nominalTotalCost_).finished() );
		iterationISE1_.push_back( (Eigen::VectorXd(1) << nominalConstraint1ISE_).finished() );

		// loop variables
		relCost = fabs(nominalTotalCost_-costCashed);
		relConstraint1ISE = fabs(nominalConstraint1ISE_-constraint1ISECashed);
		isConstraint1Satisfied  = nominalConstraint1ISE_<=options_.minAbsConstraint1ISE_ || relConstraint1ISE<=options_.minRelConstraint1ISE_;
		isCostFunctionConverged = learningRateStar_==0 || relCost<=options_.minRelCostGSLQP_;
		isOptimizationConverged = isCostFunctionConverged==true && isConstraint1Satisfied==true;

		// display
		if (options_.dispayGSLQP_)  {
			// calculates type-2 constraint ISE and maximum norm
			constraint2MaxNorm_ = calculateConstraintISE(nominalTimeTrajectoriesStock_, nc2TrajectoriesStock_, HvTrajectoryStock_, nominalConstraint2ISE_);

			std::cerr << "optimization cost:         " << nominalTotalCost_ << std::endl;
			std::cerr << "constraint type-1 ISE:     " << nominalConstraint1ISE_ << std::endl;
			std::cerr << "constraint type-1 MaxNorm: " << constraint1MaxNorm_ << std::endl;
			std::cerr << "constraint type-2 ISE:     " << nominalConstraint2ISE_ << std::endl;
			std::cerr << "constraint type-2 MaxNorm: " << constraint2MaxNorm_ << std::endl;
			std::cerr << "final constraint type-2: 	 ";
			for(size_t i=initActiveSubsystem_; i<=finalActiveSubsystem_; i++)
				std::cerr << "[" << i  << "]: " << HvFinalStock_[i].head(nc2FinalStock_[i]).transpose() << ",  ";
			std::cerr << std::endl;
		}

		// increament iteration counter
		iteration_++;

	}  // end of while loop

//	// end profile
//	ProfilerStop();

	/*
	 * solve Sequential Riccati Equations with learningRate 0.0,
	 * calculate the nominal co-state,
	 * add the deleted controller parts
	 */
//	runExit();

	// adding the deleted controller parts
	for (size_t i=0; i<initActiveSubsystem_; i++)
		nominalControllersStock_[i].swap(deletedcontrollersStock_[i]);

	if (deletedcontrollersStock_[initActiveSubsystem_].time_.empty()==false) {

		nominalControllersStock_[initActiveSubsystem_].swap(deletedcontrollersStock_[initActiveSubsystem_]);

		for (size_t k=0; k<deletedcontrollersStock_[initActiveSubsystem_].time_.size(); k++) {
			nominalControllersStock_[initActiveSubsystem_].time_.push_back(deletedcontrollersStock_[initActiveSubsystem_].time_[k]);
			nominalControllersStock_[initActiveSubsystem_].uff_.push_back(deletedcontrollersStock_[initActiveSubsystem_].uff_[k]);
			nominalControllersStock_[initActiveSubsystem_].k_.push_back(deletedcontrollersStock_[initActiveSubsystem_].k_[k]);
		}
	}

//	// set rewind flag to false
	isRewinded_ = false;

	// display
	if (options_.dispayGSLQP_  || options_.displayShortSummary_)  {
		std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr <<   "+++++++ SLQP solver is terminated +++++++" << std::endl;
		std::cerr <<   "+++++++++++++++++++++++++++++++++++++++++" << std::endl;
		std::cerr << "Number of Iterations:      " <<  iteration_ << " out of " << options_.maxIterationGSLQP_ << std::endl;
		std::cerr << "optimization cost:         " << nominalTotalCost_ << std::endl;
		std::cerr << "constraint type-1 ISE:     " << nominalConstraint1ISE_ << std::endl;
		std::cerr << "constraint type-1 MaxNorm: " << constraint1MaxNorm_ << std::endl;
		std::cerr << "constraint type-2 ISE:     " << nominalConstraint2ISE_ << std::endl;
		std::cerr << "constraint type-2 MaxNorm: " << constraint2MaxNorm_ << std::endl;
		std::cerr << "final constraint type-2: 	 ";
		for(size_t i=initActiveSubsystem_; i<=finalActiveSubsystem_; i++)
			std::cerr << "[" << i  << "]: " << HvFinalStock_[i].head(nc2FinalStock_[i]).transpose() << ",  ";
		std::cerr << std::endl;
		if (isOptimizationConverged) {
			if (learningRateStar_==0)
				std::cerr << "SLQP successfully terminates as learningRate reduced to zero." << std::endl;
			else
				std::cerr << "SLQP successfully terminates as cost relative change (relCost=" << relCost <<") reached to the minimum value." << std::endl;

			if (nominalConstraint1ISE_<=options_.minAbsConstraint1ISE_)
				std::cerr << "Type-1 constraint absolute ISE (absConstraint1ISE=" << nominalConstraint1ISE_ << ") reached to the minimum value." << std::endl;
			else
				std::cerr << "Type-1 constraint relative ISE (relConstraint1ISE=" << relConstraint1ISE << ") reached to the minimum value." << std::endl;
		} else
			std::cerr << "Maximum number of iterations has reached." << std::endl;

		std::cerr << std::endl;
	}

}

}  // ocs2 namespace



