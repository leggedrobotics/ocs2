/*
 * Implementation of GSLQP.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */


namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::rollout(const scalar_t& initTime,
		const state_vector_t& initState,
		const scalar_t& finalTime,
		const controller_array_t& controllersStock,
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		control_vector_array2_t& inputTrajectoriesStock)  {

	slqpPtr_->rollout(initTime, initState, finalTime,
			controllersStock, timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock);
}

/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::rollout(const scalar_t& initTime,
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
		constraint2_vector_array_t& HvFinalStock)  {

	slqpPtr_->rollout(initTime, initState, finalTime,
			controllersStock, timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock,
			nc1TrajectoriesStock, EvTrajectoryStock,
			nc2TrajectoriesStock, HvTrajectoryStock, nc2FinalStock, HvFinalStock);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateCostFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const control_vector_array2_t& inputTrajectoriesStock,
		scalar_t& totalCost)  {

	slqpPtr_->calculateCostFunction(timeTrajectoriesStock, stateTrajectoriesStock, inputTrajectoriesStock,
			totalCost);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateMeritFunction(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t> >& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoryStock,
		const std::vector<std::vector<Eigen::VectorXd>>&  lagrangeTrajectoriesStock,
		const scalar_t& totalCost,
		scalar_t& meritFuntionValue,
		scalar_t& constraintISE)  {

	slqpPtr_->calculateMeritFunction(timeTrajectoriesStock, nc1TrajectoriesStock, EvTrajectoryStock, lagrangeTrajectoriesStock, totalCost,
			meritFuntionValue, constraintISE);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
double GSLQP<STATE_DIM, INPUT_DIM>::calculateConstraintISE(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const std::vector<std::vector<size_t>>& nc1TrajectoriesStock,
		const constraint1_vector_array2_t& EvTrajectoriesStock,
		scalar_t& constraintISE)  {

	return slqpPtr_->calculateConstraintISE(timeTrajectoriesStock, nc1TrajectoriesStock, EvTrajectoriesStock,
			constraintISE);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::transformLocalValueFuntionDerivative2Global()  {

	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > nominalStateFunc;

	for (int i=0; i<numSubsystems_; i++) {

		nominalStateFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		nominalStateFunc.setData( &(slqpPtr_->nominalStateTrajectoriesStock_[i]) );

		for (int k=0; k<slqpPtr_->SsTimeTrajectoryStock_[i].size(); k++) {

			state_vector_t nominalState;
			nominalStateFunc.interpolate(slqpPtr_->SsTimeTrajectoryStock_[i][k], nominalState);

			for (int j=0; j<numSubsystems_-1; j++)  {

				nablasTrajectoryStock_[i][k][j] += - nominalState.transpose()*nablaSvTrajectoryStock_[i][k][j] +
						0.5*nominalState.transpose()*nablaSmTrajectoryStock_[i][k][j]*nominalState;
				nablaSvTrajectoryStock_[i][k][j]+= - nablaSmTrajectoryStock_[i][k][j]*nominalState;
			}  // end of j loop
		}  // end of k loop
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getRolloutSensitivity2SwitchingTime(
		std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		std::vector<nabla_state_matrix_array_t>& sensitivityStateTrajectoriesStock,
		std::vector<nabla_input_matrix_array_t>& sensitivityInputTrajectoriesStock)  {

	sensitivityTimeTrajectoriesStock   = sensitivityTimeTrajectoryStock_;
	sensitivityStateTrajectoriesStock = nablaStateTrajectoryStock_;
	sensitivityInputTrajectoriesStock  = nablaInputTrajectoryStock_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getController(controller_array_t& controllersStock) {

	slqpPtr_->getController(controllersStock);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getValueFuntion(const scalar_t& time, const state_vector_t& state, scalar_t& valueFuntion)  {

	slqpPtr_->getValueFuntion(time, state, valueFuntion);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getCostFuntion(scalar_t& costFunction, scalar_t& constraintISE)  {

	slqpPtr_->getCostFuntion(costFunction, constraintISE);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getValueFuntionDerivative(const state_vector_t& initState,
		Eigen::VectorXd& valueFuntionDerivative)  {

	valueFuntionDerivative.resize(numSubsystems_-1);

	for (int j=0; j<numSubsystems_-1; j++)  {

		state_matrix_t dSm  = nablaSmTrajectoryStock_[0][0][j];
		state_vector_t dSv = nablaSvTrajectoryStock_[0][0][j];
		eigen_scalar_t ds   = nablasTrajectoryStock_[0][0][j];

		valueFuntionDerivative(j) = (ds + initState.transpose()*dSv + 0.5*initState.transpose()*dSm*initState).eval()(0);
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getCostFuntionDerivative(Eigen::VectorXd& costFunctionDerivative)  {

	costFunctionDerivative = nominalCostFuntionDerivative_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::getNominalTrajectories(std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
		state_vector_array2_t& nominalStateTrajectoriesStock, control_vector_array2_t& nominalInputTrajectoriesStock)   {

	slqpPtr_->getNominalTrajectories(nominalTimeTrajectoriesStock, nominalStateTrajectoriesStock, nominalInputTrajectoriesStock);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::solveSensitivityRiccatiEquations(const scalar_t& learningRate)  {

	// final value for the last Riccati equations
	Eigen::VectorXd allSsFinal = Eigen::VectorXd::Zero( (numSubsystems_-1) * SensitivityRiccatiEquations_t::S_DIM_ );

	for (int i=numSubsystems_-1; i>=0; i--) {

		// subsystem terminal costs
		state_matrix_array_t nablaQmFinal(numSubsystems_-1, state_matrix_t::Zero());
		state_vector_array_t nablaQvFinal(numSubsystems_-1, state_vector_t::Zero());
		eigen_scalar_array_t nablaqFinal(numSubsystems_-1, eigen_scalar_t::Zero());

		for (int j=0; j<numSubsystems_-1; j++) {
			nablaQvFinal[j] = nablaQvFinalStock_[i].col(j);
			nablaqFinal[j]  = nablaqFinalStock_[i].col(j);
		}  // end of j loop

		Eigen::VectorXd allQsFinal;
		SensitivityRiccatiEquations_t::convert2Vector(numSubsystems_, nablaQmFinal, nablaQvFinal, nablaqFinal, allQsFinal);
		allSsFinal += allQsFinal;

		// set data for Riccati equations
		std::shared_ptr<SensitivityRiccatiEquations_t> sensitivityRiccatiEquationsPtr( new SensitivityRiccatiEquations_t());
		sensitivityRiccatiEquationsPtr->setData(numSubsystems_, learningRate,
				i, switchingTimes_[i], switchingTimes_[i+1],
				&slqpPtr_->SsTimeTrajectoryStock_[i], &slqpPtr_->SmTrajectoryStock_[i], &slqpPtr_->SvTrajectoryStock_[i],
				&slqpPtr_->nominalTimeTrajectoriesStock_[i],
				&slqpPtr_->AmTrajectoryStock_[i], &slqpPtr_->BmTrajectoryStock_[i],
				&slqpPtr_->qTrajectoryStock_[i], &slqpPtr_->QvTrajectoryStock_[i], &slqpPtr_->QmTrajectoryStock_[i],
				&slqpPtr_->RvTrajectoryStock_[i], &slqpPtr_->RmInverseTrajectoryStock_[i], &slqpPtr_->RmTrajectoryStock_[i], &slqpPtr_->PmTrajectoryStock_[i],
				&sensitivityTimeTrajectoryStock_[i], &nablaqTrajectoryStock_[i], &nablaQvTrajectoryStock_[i], &nablaRvTrajectoryStock_[i]);

		// normalized integration time based on SsTimeTrajectoryStock_
		int N = slqpPtr_->SsTimeTrajectoryStock_[i].size();
		scalar_array_t normalizedTimeTrajectory(N);
		for (int k=0; k<N; k++)
			normalizedTimeTrajectory[N-1-k] = (slqpPtr_->SsTimeTrajectoryStock_[i][k]-switchingTimes_[i+1])/(switchingTimes_[i]-switchingTimes_[i+1]);

		// integrating the Riccati equations
		ODE45<Eigen::Dynamic> ode45(sensitivityRiccatiEquationsPtr);

		std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> allSsTrajectory;
		ode45.integrate(allSsFinal, normalizedTimeTrajectory, allSsTrajectory, 1e-3, options_.AbsTolODE_, options_.RelTolODE_);

		// construct 'Sm', 'Sv', and 's'
		nablaSmTrajectoryStock_[i].resize(N);
		nablaSvTrajectoryStock_[i].resize(N);
		nablasTrajectoryStock_[i].resize(N);
		for (int k=0; k<N; k++)
			SensitivityRiccatiEquations_t::convert2Matrix(numSubsystems_, allSsTrajectory[N-1-k],
					nablaSmTrajectoryStock_[i][k], nablaSvTrajectoryStock_[i][k], nablasTrajectoryStock_[i][k]);

		// reset the final value for next Riccati equation
		allSsFinal = allSsTrajectory.back();

	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateSensitivityControllerFeedback(sensitivity_controller_array_t& sensitivityControllersStock) {


	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     RmInverseFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > CmProjectedFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> >     DmProjectedFunc;

	for (int i=0; i<numSubsystems_; i++) {

		BmFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		BmFunc.setData( &(slqpPtr_->BmTrajectoryStock_[i]) );

		PmFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(slqpPtr_->PmTrajectoryStock_[i]) );

		RmInverseFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		RmInverseFunc.setData( &(slqpPtr_->RmInverseTrajectoryStock_[i]) );

		CmProjectedFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		CmProjectedFunc.setData( &(slqpPtr_->CmProjectedTrajectoryStock_[i]) );

		DmProjectedFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		DmProjectedFunc.setData( &(slqpPtr_->DmProjectedTrajectoryStock_[i]) );

		sensitivityControllersStock[i].time_ = slqpPtr_->SsTimeTrajectoryStock_[i];

		size_t N = slqpPtr_->SsTimeTrajectoryStock_[i].size();
		sensitivityControllersStock[i].k_.resize(N);

		for (int k=0; k<N; k++) {

			const double& time = slqpPtr_->SsTimeTrajectoryStock_[i][k];

			control_gain_matrix_t Bm;
			BmFunc.interpolate(time, Bm);
			size_t greatestLessTimeStampIndex = BmFunc.getGreatestLessTimeStampIndex();
			control_feedback_t Pm;
			PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(time, RmInverse, greatestLessTimeStampIndex);
			control_feedback_t CmProjected;
			CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);
			control_matrix_t DmProjected;
			DmProjectedFunc.interpolate(time, DmProjected, greatestLessTimeStampIndex);

			control_feedback_t Lm  = RmInverse * (Pm + Bm.transpose()*slqpPtr_->SmTrajectoryStock_[i][k]);
			control_matrix_t DmNullProjection = control_matrix_t::Identity()-DmProjected;
			sensitivityControllersStock[i].k_[k] = -DmNullProjection*Lm - CmProjected;

			// checking the numerical stability of the controller parameters
			try {
				if (sensitivityControllersStock[i].k_[k] != sensitivityControllersStock[i].k_[k])
					throw std::runtime_error("sensitivityController feedback gains are unstable.");
			}
			catch(const std::exception& error)  {
				std::cerr << "what(): " << error.what() << " at time " << sensitivityControllersStock[i].time_[k] << " [sec]." << std::endl;
			}

		}  // end of k loop
	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateLQSensitivityControllerForward(
		sensitivity_controller_array_t& sensitivityControllersStock)  {

	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmInverseFunc;
	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;
	LinearInterpolation<nabla_input_matrix_t> nabla_RvFunc;

	for (int i=0; i<numSubsystems_; i++) {

		// set data
		BmFunc.setTimeStamp(&slqpPtr_->nominalTimeTrajectoriesStock_[i]);
		BmFunc.setData(&slqpPtr_->BmTrajectoryStock_[i]);
		RmInverseFunc.setTimeStamp(&slqpPtr_->nominalTimeTrajectoriesStock_[i]);
		RmInverseFunc.setData(&slqpPtr_->RmInverseTrajectoryStock_[i]);
		nabla_RvFunc.setTimeStamp(&sensitivityTimeTrajectoryStock_[i]);
		nabla_RvFunc.setData(&nablaRvTrajectoryStock_[i]);

		// resizing the
		size_t N = sensitivityControllersStock[i].time_.size();
		sensitivityControllersStock[i].uff_.resize(N);

		for (size_t k=0; k<N; k++) {

			// time
			const double& t = sensitivityControllersStock[i].time_[k];

			// Bm
			control_gain_matrix_t Bm;
			BmFunc.interpolate(t, Bm);
			size_t greatestLessTimeStampIndex = BmFunc.getGreatestLessTimeStampIndex();
			// RmInverse
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(t, RmInverse, greatestLessTimeStampIndex);

			// nabla_Rv
			nabla_input_matrix_t nabla_Rv(INPUT_DIM,numSubsystems_-1);
			nabla_RvFunc.interpolate(t, nabla_Rv);

			// nabla_sv
			Eigen::MatrixXd nabla_Sv(STATE_DIM,numSubsystems_-1);
			for (size_t j=0; j<numSubsystems_-1; j++)
				nabla_Sv.col(j) = nablaSvTrajectoryStock_[i][k][j];

			sensitivityControllersStock[i].uff_[k] = -RmInverse*(nabla_Rv+Bm.transpose()*nabla_Sv);
		}
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateBVPSensitivityControllerForward(
		const size_t& switchingTimeIndex,
		const state_vector_array2_t& SvTrajectoriesStock,
		sensitivity_controller_array_t& sensitivityControllersStock)  {

	if (switchingTimeIndex < 1)  throw std::runtime_error("The initial switching time (startTime) is fixed and cost function derivative is not defined.");

	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmInverseFunc;
	LinearInterpolation<control_gain_matrix_t,Eigen::aligned_allocator<control_gain_matrix_t> > BmFunc;

	sensitivityControllersStock.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		// set data
		BmFunc.setTimeStamp(&slqpPtr_->nominalTimeTrajectoriesStock_[i]);
		BmFunc.setData(&slqpPtr_->BmTrajectoryStock_[i]);
		RmInverseFunc.setTimeStamp(&slqpPtr_->nominalTimeTrajectoriesStock_[i]);
		RmInverseFunc.setData(&slqpPtr_->RmInverseTrajectoryStock_[i]);

		// resizing the
		size_t N = slqpPtr_->SsTimeTrajectoryStock_[i].size();
		sensitivityControllersStock[i].uff_.resize(N);

		for (size_t k=0; k<N; k++) {

			// time
			const double& t = slqpPtr_->SsTimeTrajectoryStock_[i][k];

			// Bm
			control_gain_matrix_t Bm;
			BmFunc.interpolate(t, Bm);
			size_t greatestLessTimeStampIndex = BmFunc.getGreatestLessTimeStampIndex();
			// RmInverse
			control_matrix_t RmInverse;
			RmInverseFunc.interpolate(t, RmInverse, greatestLessTimeStampIndex);

//			if (switchingTimeIndex-1==0)
			sensitivityControllersStock[i].uff_[k].resize(INPUT_DIM, numSubsystems_-1);

			sensitivityControllersStock[i].uff_[k].col(switchingTimeIndex-1) = -RmInverse*Bm.transpose()*SvTrajectoriesStock[i][k]; //FIXME

		}  // end of k loop
	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::rolloutSensitivity2SwitchingTime(
		const sensitivity_controller_array_t& sensitivityControllersStock,
		std::vector<scalar_array_t>& sensitivityTimeTrajectoryStock,
		std::vector<nabla_state_matrix_array_t>& nablaStateTrajectoryStock,
		std::vector<nabla_input_matrix_array_t>& nablaInputTrajectoryStock)  {

	std::shared_ptr<RolloutSensitivityEquations_t> rolloutSensitivityEquationsPtr( new RolloutSensitivityEquations_t() );
	Eigen::VectorXd nabla_XmInit;
	RolloutSensitivityEquations_t::convert2Vector(numSubsystems_, Eigen::MatrixXd::Zero(STATE_DIM,numSubsystems_-1), nabla_XmInit);

	sensitivityTimeTrajectoryStock.resize(numSubsystems_);
	nablaStateTrajectoryStock.resize(numSubsystems_);
	nablaInputTrajectoryStock.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		// initialize subsystem i
		slqpPtr_->getSubsystemDynamicsPtrStock()[i]->initializeModel(systemStockIndexes_, switchingTimes_,
				slqpPtr_->nominalStateTrajectoriesStock_[i].front(), i, "GSLQP");

		rolloutSensitivityEquationsPtr->setData(i, switchingTimes_, &sensitivityControllersStock[i],
				&slqpPtr_->nominalTimeTrajectoriesStock_[i], &nominalStateTimeDerivativeTrajectoriesStock_[i],
				&slqpPtr_->AmTrajectoryStock_[i], &slqpPtr_->BmTrajectoryStock_[i]);

		// integrating
		scalar_array_t normalizedSensitivityTimeTrajectory;
		std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> sensitivityStateTrajectory;
		ODE45<Eigen::Dynamic> ode45(rolloutSensitivityEquationsPtr);
		ode45.integrate(nabla_XmInit, 0.0, 1.0, sensitivityStateTrajectory, normalizedSensitivityTimeTrajectory,
				1e-3, options_.AbsTolODE_, options_.RelTolODE_);

		// denormalizing time and constructing SensitivityStateTrajectory and computing control trajectory sensitivity for subsystem i
		int N = normalizedSensitivityTimeTrajectory.size();
		sensitivityTimeTrajectoryStock[i].resize(N);
		nablaStateTrajectoryStock[i].resize(N);
		nablaInputTrajectoryStock[i].resize(N);
		for (int k=0; k<N; k++) {

			sensitivityTimeTrajectoryStock[i][k] = switchingTimes_[i] + (switchingTimes_[i+1]-switchingTimes_[i])*normalizedSensitivityTimeTrajectory[k];
			RolloutSensitivityEquations_t::convert2Matrix(numSubsystems_, sensitivityStateTrajectory[k], nablaStateTrajectoryStock[i][k]);
			rolloutSensitivityEquationsPtr->computeInputSensitivity(sensitivityTimeTrajectoryStock[i][k], nablaStateTrajectoryStock[i][k],
					nablaInputTrajectoryStock[i][k]);
		}

		// reset the initial state
		nabla_XmInit = sensitivityStateTrajectory.back();

	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::approximateNominalLQPSensitivity2SwitchingTime() {

	// calculate nabla_q, nabla_Qv, nabla_Rv
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t> > QvFunc;
	LinearInterpolation<state_matrix_t,Eigen::aligned_allocator<state_matrix_t> > QmFunc;
	LinearInterpolation<control_vector_t,Eigen::aligned_allocator<control_vector_t> > RvFunc;
	LinearInterpolation<control_matrix_t,Eigen::aligned_allocator<control_matrix_t> > RmFunc;
	LinearInterpolation<control_feedback_t,Eigen::aligned_allocator<control_feedback_t> > PmFunc;

	nablaqTrajectoryStock_.resize(numSubsystems_);
	nablaQvTrajectoryStock_.resize(numSubsystems_);
	nablaRvTrajectoryStock_.resize(numSubsystems_);
	nablaEvTrajectoryStock_.resize(numSubsystems_);
	nablaqFinalStock_.resize(numSubsystems_);
	nablaQvFinalStock_.resize(numSubsystems_);

	for (int i=0; i<numSubsystems_; i++) {

		QvFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		QvFunc.setData( &(slqpPtr_->QvTrajectoryStock_[i]) );
		QmFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		QmFunc.setData( &(slqpPtr_->QmTrajectoryStock_[i]) );
		RvFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		RvFunc.setData( &(slqpPtr_->RvTrajectoryStock_[i]) );
		RmFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		RmFunc.setData( &(slqpPtr_->RmTrajectoryStock_[i]) );
		PmFunc.setTimeStamp( &(slqpPtr_->nominalTimeTrajectoriesStock_[i]) );
		PmFunc.setData( &(slqpPtr_->PmTrajectoryStock_[i]) );

		int N = sensitivityTimeTrajectoryStock_[i].size();
		nablaqTrajectoryStock_[i].resize(N);
		nablaQvTrajectoryStock_[i].resize(N);
		nablaRvTrajectoryStock_[i].resize(N);

		for (int k=0; k<N; k++) {

			control_matrix_t Rm;
			RmFunc.interpolate(sensitivityTimeTrajectoryStock_[i][k], Rm);
			size_t greatestLessTimeStampIndex = RmFunc.getGreatestLessTimeStampIndex();
			state_vector_t Qv;
			QvFunc.interpolate(sensitivityTimeTrajectoryStock_[i][k], Qv, greatestLessTimeStampIndex);
			state_matrix_t Qm;
			QmFunc.interpolate(sensitivityTimeTrajectoryStock_[i][k], Qm, greatestLessTimeStampIndex);
			control_vector_t Rv;
			RvFunc.interpolate(sensitivityTimeTrajectoryStock_[i][k], Rv, greatestLessTimeStampIndex);
			control_feedback_t Pm;
			PmFunc.interpolate(sensitivityTimeTrajectoryStock_[i][k], Pm, greatestLessTimeStampIndex);

			nablaqTrajectoryStock_[i][k]  = Qv.transpose()*nablaStateTrajectoryStock_[i][k] + Rv.transpose()*nablaInputTrajectoryStock_[i][k];
			nablaQvTrajectoryStock_[i][k] = Qm*nablaStateTrajectoryStock_[i][k] + Pm.transpose()*nablaInputTrajectoryStock_[i][k];
			nablaRvTrajectoryStock_[i][k] = Pm*nablaStateTrajectoryStock_[i][k] + Rm*nablaInputTrajectoryStock_[i][k];
		}

		state_vector_t Qv = slqpPtr_->QvFinalStock_[i];
		state_matrix_t Qm  = slqpPtr_->QmFinalStock_[i];

		nablaqFinalStock_[i]  = Qv.transpose()*nablaStateTrajectoryStock_[i].back();
		nablaQvFinalStock_[i] = Qm*nablaStateTrajectoryStock_[i].back();
	}  // end of i loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateStateTimeDerivative()  {

	nominalStateTimeDerivativeTrajectoriesStock_.resize(numSubsystems_);

	for (size_t i=0; i<numSubsystems_; i++) {

		slqpPtr_->getSubsystemDynamicsPtrStock()[i]->initializeModel(systemStockIndexes_, switchingTimes_,
				slqpPtr_->nominalStateTrajectoriesStock_[i].front(), i, "GSLQP");

		size_t N = slqpPtr_->nominalTimeTrajectoriesStock_[i].size();
		nominalStateTimeDerivativeTrajectoriesStock_[i].resize(N);

		for (size_t k=0; k<N; k++) {
			const scalar_t& 	    t = slqpPtr_->nominalTimeTrajectoriesStock_[i][k];
			const state_vector_t&   x = slqpPtr_->nominalStateTrajectoriesStock_[i][k];
			const control_vector_t& u = slqpPtr_->nominalInputTrajectoriesStock_[i][k];
			slqpPtr_->getSubsystemDynamicsPtrStock()[i]->computeDerivative(t, x, u, nominalStateTimeDerivativeTrajectoriesStock_[i][k]);

		}  // end of k loop
	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::solveSensitivityBVP(
		const size_t& switchingTimeIndex,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_matrix_array2_t& MmTrajectoriesStock,
		state_vector_array2_t& SvTrajectoriesStock)  {

	if (switchingTimeIndex < 1)  throw std::runtime_error("The initial switching time (startTime) is fixed and cost function derivative is not defined.");

	// calculate the BVP coefficients
	state_vector_array_t bvpGvPositivetraTrajectory(slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex-1].size());
	state_vector_array_t bvpQvPositivetraTrajectory(slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex-1].size());
	state_vector_array_t bvpGvNegativetraTrajectory(slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex].size());
	state_vector_array_t bvpQvNegativetraTrajectory(slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex].size());

	const double scalingFactor = 1/(switchingTimes_[switchingTimeIndex]-switchingTimes_[switchingTimeIndex-1]);

	for (size_t k=0; k<slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex-1].size(); k++) {
		bvpGvPositivetraTrajectory[k] = scalingFactor * nominalStateTimeDerivativeTrajectoriesStock_[switchingTimeIndex-1][k];
		bvpQvPositivetraTrajectory[k] = scalingFactor * (slqpPtr_->QvTrajectoryStock_[switchingTimeIndex-1][k]+
				slqpPtr_->AmTrajectoryStock_[switchingTimeIndex-1][k].transpose()*slqpPtr_->nominalcostateTrajectoriesStock_[switchingTimeIndex-1][k]);
	}

	for (size_t k=0; k<slqpPtr_->nominalTimeTrajectoriesStock_[switchingTimeIndex].size(); k++) {
		bvpGvNegativetraTrajectory[k] = -scalingFactor * nominalStateTimeDerivativeTrajectoriesStock_[switchingTimeIndex][k];
		bvpQvNegativetraTrajectory[k] = -scalingFactor * (slqpPtr_->QvTrajectoryStock_[switchingTimeIndex][k]+
				slqpPtr_->AmTrajectoryStock_[switchingTimeIndex][k].transpose()*slqpPtr_->nominalcostateTrajectoriesStock_[switchingTimeIndex][k]);
	}

	SolveBVP<STATE_DIM, INPUT_DIM> bvpSolver;
	state_vector_t SvFinal = state_vector_t::Zero();
	state_matrix_t  MmFinal =  state_matrix_t::Zero();

	for (int i=numSubsystems_-1; i>=0; i--) {

		MmFinal += slqpPtr_->QmFinalStock_[i];

		const state_vector_array_t* GvPtr;
		const state_vector_array_t* QvPtr;
		if (i==switchingTimeIndex-1) {
			GvPtr = &bvpGvPositivetraTrajectory;
			QvPtr = &bvpQvPositivetraTrajectory;
		} else if (i==switchingTimeIndex) {
			GvPtr = &bvpGvNegativetraTrajectory;
			QvPtr = &bvpQvNegativetraTrajectory;
		} else {
			GvPtr = NULL;
			QvPtr = NULL;
		}

		// set the general BVP solver coefficient
		bvpSolver.setData(&slqpPtr_->nominalTimeTrajectoriesStock_[i],
				&slqpPtr_->AmTrajectoryStock_[i], NULL,  &slqpPtr_->BmTrajectoryStock_[i], GvPtr,
				QvPtr, &slqpPtr_->QmTrajectoryStock_[i], &slqpPtr_->PmTrajectoryStock_[i],
				NULL, &slqpPtr_->RmTrajectoryStock_[i], &slqpPtr_->RmInverseTrajectoryStock_[i]);


		// solve BVP for the given time trajectory
		bvpSolver.solve(timeTrajectoriesStock[i], SvFinal, MmFinal,
				MmTrajectoriesStock[i], SvTrajectoriesStock[i],
				options_.AbsTolODE_, options_.RelTolODE_);

		//set the final value of the previous subsystem solver to the starting time value of the current subsystem's solution
		SvFinal = SvTrajectoriesStock[i].front();
		MmFinal = MmTrajectoriesStock[i].front();

//		for (size_t k=0; k<timeTrajectoriesStock[i].size(); k++)
//			if (!MmTrajectoriesStock[i][k].isApprox(slqpPtr_->SmTrajectoryStock_[i][k], 1e-3)) {
//				std::cerr << "In solveSensitivityBVP, Mm and Sm do not match." << std::endl;
//				std::cerr << "Mm[" << i << "][" << k << "]\n" <<  MmTrajectoriesStock[i][k] << std::endl;
//				std::cerr << "Sm[" << i << "][" << k << "]\n" <<  slqpPtr_->SmTrajectoryStock_[i][k] << std::endl;
//			}

	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::calculateBVPCostFunctionDerivative(
		Eigen::VectorXd& costFunctionDerivative)  {

	// final time
	costFunctionDerivative = Eigen::VectorXd::Zero(numSubsystems_-1);

	for (size_t i=0; i<numSubsystems_; i++) {

		LinearInterpolation<eigen_scalar_t, Eigen::aligned_allocator<eigen_scalar_t> > qFunc(
				&slqpPtr_->nominalTimeTrajectoriesStock_[i], &slqpPtr_->qTrajectoryStock_[i]);
		LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > QvFunc(
				&slqpPtr_->nominalTimeTrajectoriesStock_[i], &slqpPtr_->QvTrajectoryStock_[i]);
		LinearInterpolation<control_vector_t, Eigen::aligned_allocator<control_vector_t> > RvFunc(
				&slqpPtr_->nominalTimeTrajectoriesStock_[i], &slqpPtr_->RvTrajectoryStock_[i]);

		Eigen::VectorXd previousIntermediatecostFunctionDev(numSubsystems_-1);
		Eigen::VectorXd currentIntermediatecostFunctionDev(numSubsystems_-1);

		for (int k=0; k<sensitivityTimeTrajectoryStock_[i].size(); k++) {

			const double& t = sensitivityTimeTrajectoryStock_[i][k];

			state_vector_t Qv;
			QvFunc.interpolate(t, Qv);
			size_t greatestLessTimeStampIndex = QvFunc.getGreatestLessTimeStampIndex();
			control_vector_t Rv;
			RvFunc.interpolate(t, Rv, greatestLessTimeStampIndex);
			eigen_scalar_t q;
			qFunc.interpolate(t, q, greatestLessTimeStampIndex);

			Eigen::VectorXd coeff(numSubsystems_-1);
			for (int j=1; j<numSubsystems_; j++)
				if (i==j-1)
					coeff(j-1) = +1.0;
				else if (i==j)
					coeff(j-1) = -1.0;
				else
					coeff(j-1) = 0.0;

			if (k>0)
				previousIntermediatecostFunctionDev = currentIntermediatecostFunctionDev;

			currentIntermediatecostFunctionDev = coeff*q/(switchingTimes_[i+1]-switchingTimes_[i]) +
					(nablaStateTrajectoryStock_[i][k].transpose()*Qv + nablaInputTrajectoryStock_[i][k].transpose()*Rv);

			if (k>0)
				costFunctionDerivative += 0.5*(sensitivityTimeTrajectoryStock_[i][k]-sensitivityTimeTrajectoryStock_[i][k-1]) *
				(currentIntermediatecostFunctionDev+previousIntermediatecostFunctionDev);

		}  // end of k loop

		// subsystem final cost
		costFunctionDerivative += nablaStateTrajectoryStock_[i].back().transpose() * slqpPtr_->QvFinalStock_[i];

	}  // end of i loop

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::runLQBasedMethod()  {

	nominalSensitivityControllersStock_.resize(numSubsystems_);
	nablasTrajectoryStock_.resize(numSubsystems_);
	nablaSvTrajectoryStock_.resize(numSubsystems_);
	nablaSmTrajectoryStock_.resize(numSubsystems_);

	// calculate state time derivative
	calculateStateTimeDerivative();

	// calculate sensitivity controller feedback part
	calculateSensitivityControllerFeedback(nominalSensitivityControllersStock_);
	// set sensitivity controller feedforward part to zero
	for (size_t i=0; i<numSubsystems_; i++) {
		size_t N = nominalSensitivityControllersStock_[i].time_.size();
		nominalSensitivityControllersStock_[i].uff_.resize(N);
		for (size_t k=0; k<N; k++)  nominalSensitivityControllersStock_[i].uff_[k].setZero();
	}


	for (size_t j=0; j<3; j++) {

		// calculate nominal rollout sensitivity to switching times
		rolloutSensitivity2SwitchingTime(nominalSensitivityControllersStock_,
				sensitivityTimeTrajectoryStock_, nablaStateTrajectoryStock_, nablaInputTrajectoryStock_);

		// approximate the nominal LQP sensitivity to switching times
		approximateNominalLQPSensitivity2SwitchingTime();

		// solve Riccati equations
		solveSensitivityRiccatiEquations(0.0 /*learningRateStar*/); // prevents the changes in the nominal trajectories and just update the gains

		// calculate sensitivity controller feedforward part
		calculateLQSensitivityControllerForward(nominalSensitivityControllersStock_);
	}

	// transform from local value function derivatives to global representation
	transformLocalValueFuntionDerivative2Global();

	// calculate the cost function derivatives w.r.t. switchingTimes
	getValueFuntionDerivative(slqpPtr_->nominalStateTrajectoriesStock_[0][0], nominalCostFuntionDerivative_);

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::runSweepingBVPMethod()  {

	nominalSensitivityControllersStock_.resize(numSubsystems_);
	nablasTrajectoryStock_.resize(numSubsystems_);
	nablaSvTrajectoryStock_.resize(numSubsystems_);
	nablaSmTrajectoryStock_.resize(numSubsystems_);

	// calculate state time derivative
	calculateStateTimeDerivative();

	// calculate sensitivity controller feedback part
	calculateSensitivityControllerFeedback(nominalSensitivityControllersStock_);

	// for each switching time solve BVP
//#pragma omp parallel for
	for (size_t j=1; j<numSubsystems_; j++)  {
		state_matrix_array2_t  MmTrajectoriesStock(numSubsystems_);
		state_vector_array2_t SvTrajectoriesStock(numSubsystems_);

		// solve boundary value problem of the sensitivity equations for switching time j
		solveSensitivityBVP(j, slqpPtr_->SsTimeTrajectoryStock_, MmTrajectoriesStock, SvTrajectoriesStock);

		// calculate sensitivity controller feedforward part
		calculateBVPSensitivityControllerForward(j, SvTrajectoriesStock, nominalSensitivityControllersStock_);
	} // end of j loop


	// calculate nominal rollout sensitivity to switching times
	rolloutSensitivity2SwitchingTime(nominalSensitivityControllersStock_,
			sensitivityTimeTrajectoryStock_, nablaStateTrajectoryStock_, nablaInputTrajectoryStock_);

	// calculate the cost function derivatives w.r.t. switchingTimes
	calculateBVPCostFunctionDerivative(nominalCostFuntionDerivative_);
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM>
void GSLQP<STATE_DIM, INPUT_DIM>::run(const double& initTime, const state_vector_t& initState, const double& finalTime,
		const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
		const slqp_ptr_t& slqpPtr,
		const std::vector<scalar_array_t>& desiredTimeTrajectoriesStock/*=scalar_array2_t()*/,
		const state_vector_array2_t& desiredStateTrajectoriesStock/*=state_vector_array2_t()*/)  {

	numSubsystems_ = systemStockIndexes.size();
	initTime_ = initTime;
	initState_ = initState;
	finalTime_ = finalTime;
	switchingTimes_ = switchingTimes;
	systemStockIndexes_ = systemStockIndexes;

	// make sure that the minimum difference between to successive switching times is at least options_.minSimulationTimeDuration_
	for (size_t i=0; i<numSubsystems_; i++)
		if (switchingTimes_[i+1]-switchingTimes_[i] < options_.minSimulationTimeDuration_) {
			if (i+1 == numSubsystems_)
				std::cerr << "WARNING: The minimum simulation time between the last subsystem's stratTime and finalTime should be at least "
				<< options_.minSimulationTimeDuration_ << "." << std::endl;
			switchingTimes_[i+1] = switchingTimes_[i]+options_.minSimulationTimeDuration_;
		}

	// check if the SLQP solver is provided.
	// if not: use the internal one and run it once to solve the optimal control problem
	if (slqpPtr)
		slqpPtr_ = slqpPtr;
	else {
		throw std::runtime_error("An SLQ solver should be provided.");
	}

	// display
	if (options_.dispayGSLQP_)  std::cerr << "\n#### Calculating cost function sensitivity ..." << std::endl;

	// use the LQ-based method or Sweeping-BVP method
	if (options_.useLQForDerivatives_==true)
		runLQBasedMethod();
	else
		runSweepingBVPMethod();


//	{
//		SolveBVP<STATE_DIM, INPUT_DIM> bvpSolver;
//		state_matrix_array2_t MmTrajectoriesStock(numSubsystems_);
//		state_vector_array2_t SvTrajectoriesStock(numSubsystems_);
//		state_vector_t SvFinal = slqpPtr_->QvFinal_;
//		state_matrix_t MmFinal = slqpPtr_->QmFinal_;
//		for (int i=numSubsystems_-1; i>=0; i--) {
//
//			bvpSolver.setData(&slqpPtr_->nominalTimeTrajectoriesStock_[i],
//					&slqpPtr_->AmConstrainedTrajectoryStock_[i], NULL,  &slqpPtr_->BmTrajectoryStock_[i], NULL,
//					&slqpPtr_->QvConstrainedTrajectoryStock_[i], &slqpPtr_->QmConstrainedTrajectoryStock_[i], &slqpPtr_->PmTrajectoryStock_[i],
//					&slqpPtr_->RvTrajectoryStock_[i], &slqpPtr_->RmConstrainedTrajectoryStock_[i], &slqpPtr_->RmInverseTrajectoryStock_[i]);
//
//			bvpSolver.solve(slqpPtr_->SsTimeTrajectoryStock_[i], SvFinal, MmFinal,
//					MmTrajectoriesStock[i], SvTrajectoriesStock[i],
//					options_.AbsTolODE_, options_.RelTolODE_);
//
//			SvFinal = SvTrajectoriesStock[i].front();
//			MmFinal = MmTrajectoriesStock[i].front();
//
//			for (size_t k=0; k<slqpPtr_->SsTimeTrajectoryStock_[i].size(); k++) {
//				if (!MmTrajectoriesStock[i][k].isApprox(slqpPtr_->SmTrajectoryStock_[i][k], 1e-3)) {
//					std::cout << "Mm[" << i << "][" << k << "]\n" <<  MmTrajectoriesStock[i][k] << std::endl;
//					std::cout << "Sm[" << i << "][" << k << "]\n" <<  slqpPtr_->SmTrajectoryStock_[i][k] << std::endl;
//				}
//				if (!SvTrajectoriesStock[i][k].isApprox(slqpPtr_->SvTrajectoryStock_[i][k], 1e-3)) {
//					std::cout << "Sv[" << i << "][" << k << "]\n" <<  SvTrajectoriesStock[i][k] << std::endl;
//					std::cout << "Sv[" << i << "][" << k << "]\n" <<  slqpPtr_->SvTrajectoryStock_[i][k] << std::endl;
//				}
//			}
//		}
//	}


}


} // namespace ocs2

