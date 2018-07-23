/*
 * Implementation of GSLQ.h
 *
 *  Created on: Jan 5, 2016
 *      Author: farbod
 */

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::GSLQ(slq_t& slq)
	: slqPtr_(&slq)
	, settingsPtr_(&slq.settings_)
	, logicRulesMachinePtr_(slq.logicRulesMachinePtr_)
	, numPartitions_(0)
{
	// check if the SLQ solver is provided.
	if (!slqPtr_)
		throw std::runtime_error("A pointer to a SLQ solver should be provided.");

	systemDynamicsPtr_.reset(slq.systemDynamicsPtrStock_.front()->clone());

	bvpSensitivityEquationsPtrStock_.clear();
	bvpSensitivityEquationsPtrStock_.reserve(settingsPtr_->nThreads_);
	bvpSensitivityIntegratorsPtrStock_.clear();
	bvpSensitivityIntegratorsPtrStock_.reserve(settingsPtr_->nThreads_);

	rolloutSensitivityEquationsPtrStock_.clear();
	rolloutSensitivityEquationsPtrStock_.reserve(settingsPtr_->nThreads_);
	rolloutSensitivityIntegratorsPtrStock_.clear();
	rolloutSensitivityIntegratorsPtrStock_.reserve(settingsPtr_->nThreads_);

	riccatiSensitivityEquationsPtrStock_.clear();
	riccatiSensitivityEquationsPtrStock_.reserve(settingsPtr_->nThreads_);
	riccatiSensitivityIntegratorsPtrStock_.clear();
	riccatiSensitivityIntegratorsPtrStock_.reserve(settingsPtr_->nThreads_);

	for (size_t i=0; i<settingsPtr_->nThreads_; i++)  {

		typedef Eigen::aligned_allocator<bvp_sensitivity_equations_t> bvp_sensitivity_equations_alloc_t;
		bvpSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<bvp_sensitivity_equations_t, bvp_sensitivity_equations_alloc_t>(
						bvp_sensitivity_equations_alloc_t()) ) );

		typedef Eigen::aligned_allocator<rollout_sensitivity_equations_t> rollout_sensitivity_equations_alloc_t;
		rolloutSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<rollout_sensitivity_equations_t, rollout_sensitivity_equations_alloc_t>(
						rollout_sensitivity_equations_alloc_t()) ) );

		typedef Eigen::aligned_allocator<riccati_sensitivity_equations_t> riccati_sensitivity_equations_alloc_t;
		riccatiSensitivityEquationsPtrStock_.push_back( std::move(
				std::allocate_shared<riccati_sensitivity_equations_t, riccati_sensitivity_equations_alloc_t>(
						riccati_sensitivity_equations_alloc_t()) ) );

		switch(settingsPtr_->RiccatiIntegratorType_) {

		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ODE45 : {
			bvpSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<STATE_DIM>(bvpSensitivityEquationsPtrStock_.back()) );

			rolloutSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<STATE_DIM>(rolloutSensitivityEquationsPtrStock_.back()) );

			riccatiSensitivityIntegratorsPtrStock_.emplace_back (
					new ODE45<riccati_sensitivity_equations_t::S_DIM_>(
							riccatiSensitivityEquationsPtrStock_.back()) );

			break;
		}
		/* note: this case is not yet working. It would most likely work if we had an adaptive time adams-bashforth integrator */
		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::ADAMS_BASHFORTH : {
			throw std::runtime_error("This ADAMS_BASHFORTH is not implemented for Riccati Integrator.");
			break;
		}
		case DIMENSIONS::RICCATI_INTEGRATOR_TYPE::BULIRSCH_STOER : {
			bvpSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<STATE_DIM>(bvpSensitivityEquationsPtrStock_.back()) );

			rolloutSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<STATE_DIM>(rolloutSensitivityEquationsPtrStock_.back()) );

			riccatiSensitivityIntegratorsPtrStock_.emplace_back (
					new IntegratorBulirschStoer<riccati_sensitivity_equations_t::S_DIM_>(
							riccatiSensitivityEquationsPtrStock_.back()) );

			break;
		}
		default:
			throw (std::runtime_error("Riccati equations integrator type specified wrongly."));
		}

	}  // end of i loop

	// calculateBVPSensitivityControllerForward & calculateLQSensitivityControllerForward
	BmFuncStock_.resize(settingsPtr_->nThreads_);
	RmInverseFuncStock_.resize(settingsPtr_->nThreads_);
	DmProjectedFuncStock_.resize(settingsPtr_->nThreads_);
	nablaRvFuncStock_.resize(settingsPtr_->nThreads_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::setupOptimizer(const size_t& numPartitions) {

	if (numPartitions==0)
		throw std::runtime_error("The number of Partitions cannot be zero!");

	/*
	 * Data which should be copied
	 */
	// optimized controller
	nominalControllersStock_.resize(numPartitions);

	// optimized trajectories
	optimizedTimeTrajectoriesStock_.resize(numPartitions);
	optimizedEventsPastTheEndIndecesStock_.resize(numPartitions);
	optimizedStateTrajectoriesStock_.resize(numPartitions);
	optimizedInputTrajectoriesStock_.resize(numPartitions);

	// nominal trajectories
	nominalTimeTrajectoriesStock_.resize(numPartitions);
	nominalEventsPastTheEndIndecesStock_.resize(numPartitions);
	nominalStateTrajectoriesStock_.resize(numPartitions);
	nominalInputTrajectoriesStock_.resize(numPartitions);

	/*
	 * Data which can be swapped. Note that these variables should have correct size.
	 * Otherwise use setOptimizer() to construct them with correct size
	 */
	// linearized system coefficients
	AmTrajectoriesStock_.resize(numPartitions);
	BmTrajectoriesStock_.resize(numPartitions);

	nc1TrajectoriesStock_.resize(numPartitions);
	EvTrajectoriesStock_.resize(numPartitions);
	CmTrajectoriesStock_.resize(numPartitions);
	DmTrajectoriesStock_.resize(numPartitions);

	nc2TrajectoriesStock_.resize(numPartitions);
	HvTrajectoriesStock_.resize(numPartitions);
	FmTrajectoriesStock_.resize(numPartitions);
	nc2FinalStock_.resize(numPartitions);
	HvFinalStock_.resize(numPartitions);
	FmFinalStock_.resize(numPartitions);

	// cost quadratic approximation coefficients
	qFinalStock_.resize(numPartitions);
	QvFinalStock_.resize(numPartitions);
	QmFinalStock_.resize(numPartitions);

	qTrajectoriesStock_.resize(numPartitions);
	QvTrajectoriesStock_.resize(numPartitions);
	QmTrajectoriesStock_.resize(numPartitions);
	RvTrajectoriesStock_.resize(numPartitions);
	RmTrajectoriesStock_.resize(numPartitions);
	PmTrajectoriesStock_.resize(numPartitions);

	// constrained projected variables
	RmInverseTrajectoriesStock_.resize(numPartitions);
	AmConstrainedTrajectoriesStock_.resize(numPartitions);
	QmConstrainedTrajectoriesStock_.resize(numPartitions);
	QvConstrainedTrajectoriesStock_.resize(numPartitions);
	RmConstrainedTrajectoriesStock_.resize(numPartitions);
	DmDagerTrajectoriesStock_.resize(numPartitions);
	EvProjectedTrajectoriesStock_.resize(numPartitions);
	CmProjectedTrajectoriesStock_.resize(numPartitions);
	DmProjectedTrajectoriesStock_.resize(numPartitions);
	BmConstrainedTrajectoriesStock_.resize(numPartitions);
	PmConstrainedTrajectoriesStock_.resize(numPartitions);
	RvConstrainedTrajectoriesStock_.resize(numPartitions);

	// Riccati coefficients
	SsTimeTrajectoriesStock_.resize(numPartitions);
	SsNormalizedTimeTrajectoriesStock_.resize(numPartitions);
	SsNormalizedEventsPastTheEndIndecesStock_.resize(numPartitions);
	SmTrajectoriesStock_.resize(numPartitions);
	SvTrajectoriesStock_.resize(numPartitions);
	SveTrajectoriesStock_.resize(numPartitions);
	sTrajectoriesStock_.resize(numPartitions);

	/*
	 * GSLQ variables
	 */
	nominalFlowMapTrajectoriesStock_.resize(numPartitions);
	nominalCostateTrajectoriesStock_.resize(numPartitions);
	nominalLagrangianTrajectoriesStock_.resize(numPartitions);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::collectSlqData(slq_t* slqPtr) {

	/*
	 * Data which should be copied
	 */
	// initial time and state plus final time
	initTime_  = slqPtr->initTime_;
	finalTime_ = slqPtr->finalTime_;
	initState_ = slqPtr->initState_;

	// active partitions range: [initActivePartition_, finalActivePartition_]
	initActivePartition_  = slqPtr->initActivePartition_;
	finalActivePartition_ = slqPtr->finalActivePartition_;
	partitioningTimes_ = slqPtr->partitioningTimes_;

	// optimized controller
	nominalControllersStock_ = slqPtr->nominalControllersStock_;

	// optimized trajectories
	optimizedTimeTrajectoriesStock_        = slqPtr->nominalTimeTrajectoriesStock_;
	optimizedEventsPastTheEndIndecesStock_ = slqPtr->nominalEventsPastTheEndIndecesStock_;
	optimizedStateTrajectoriesStock_       = slqPtr->nominalStateTrajectoriesStock_;
	optimizedInputTrajectoriesStock_       = slqPtr->nominalInputTrajectoriesStock_;

	// nominal trajectories
	nominalTimeTrajectoriesStock_        = slqPtr->nominalPrevTimeTrajectoriesStock_;
	nominalEventsPastTheEndIndecesStock_ = slqPtr->nominalPrevEventsPastTheEndIndecesStock_;
	nominalStateTrajectoriesStock_       = slqPtr->nominalPrevStateTrajectoriesStock_;
	nominalInputTrajectoriesStock_       = slqPtr->nominalPrevInputTrajectoriesStock_;

	/*
	 * Data which can be swapped. Note that these variables should have correct size.
	 * Otherwise use setOptimizer() to construct them with correct size
	 */
	// linearized system coefficients
	AmTrajectoriesStock_.swap(slqPtr->AmTrajectoryStock_);
	BmTrajectoriesStock_.swap(slqPtr->BmTrajectoryStock_);

	nc1TrajectoriesStock_.swap(slqPtr->nc1TrajectoriesStock_);
	EvTrajectoriesStock_.swap(slqPtr->EvTrajectoryStock_);
	CmTrajectoriesStock_.swap(slqPtr->CmTrajectoryStock_);
	DmTrajectoriesStock_.swap(slqPtr->DmTrajectoryStock_);

	nc2TrajectoriesStock_.swap(slqPtr->nc2TrajectoriesStock_);
	HvTrajectoriesStock_.swap(slqPtr->HvTrajectoryStock_);
	FmTrajectoriesStock_.swap(slqPtr->FmTrajectoryStock_);
	nc2FinalStock_.swap(slqPtr->nc2FinalStock_);
	HvFinalStock_.swap(slqPtr->HvFinalStock_);
	FmFinalStock_.swap(slqPtr->FmFinalStock_);

	// cost quadratic approximation coefficients
	qFinalStock_.swap(slqPtr->qFinalStock_);
	QvFinalStock_.swap(slqPtr->QvFinalStock_);
	QmFinalStock_.swap(slqPtr->QmFinalStock_);

	qTrajectoriesStock_.swap(slqPtr->qTrajectoryStock_);
	QvTrajectoriesStock_.swap(slqPtr->QvTrajectoryStock_);
	QmTrajectoriesStock_.swap(slqPtr->QmTrajectoryStock_);
	RvTrajectoriesStock_.swap(slqPtr->RvTrajectoryStock_);
	RmTrajectoriesStock_.swap(slqPtr->RmTrajectoryStock_);
	PmTrajectoriesStock_.swap(slqPtr->PmTrajectoryStock_);

	// constrained projected variables
	RmInverseTrajectoriesStock_.swap(slqPtr->RmInverseTrajectoryStock_);
	AmConstrainedTrajectoriesStock_.swap(slqPtr->AmConstrainedTrajectoryStock_);
	QmConstrainedTrajectoriesStock_.swap(slqPtr->QmConstrainedTrajectoryStock_);
	QvConstrainedTrajectoriesStock_.swap(slqPtr->QvConstrainedTrajectoryStock_);
	RmConstrainedTrajectoriesStock_.swap(slqPtr->RmConstrainedTrajectoryStock_);
	DmDagerTrajectoriesStock_.swap(slqPtr->DmDagerTrajectoryStock_);
	EvProjectedTrajectoriesStock_.swap(slqPtr->EvProjectedTrajectoryStock_);
	CmProjectedTrajectoriesStock_.swap(slqPtr->CmProjectedTrajectoryStock_);
	DmProjectedTrajectoriesStock_.swap(slqPtr->DmProjectedTrajectoryStock_);
	BmConstrainedTrajectoriesStock_.swap(slqPtr->BmConstrainedTrajectoryStock_);
	PmConstrainedTrajectoriesStock_.swap(slqPtr->PmConstrainedTrajectoryStock_);
	RvConstrainedTrajectoriesStock_.swap(slqPtr->RvConstrainedTrajectoryStock_);

	// Riccati coefficients
	SsTimeTrajectoriesStock_.swap(slqPtr->SsTimeTrajectoryStock_);
	SsNormalizedTimeTrajectoriesStock_.swap(slqPtr->SsNormalizedTimeTrajectoryStock_);
	SsNormalizedEventsPastTheEndIndecesStock_.swap(slqPtr->SsNormalizedEventsPastTheEndIndecesStock_);
	SmTrajectoriesStock_.swap(slqPtr->SmTrajectoryStock_);
	SvTrajectoriesStock_.swap(slqPtr->SvTrajectoryStock_);
	SveTrajectoriesStock_.swap(slqPtr->SveTrajectoryStock_);
	sTrajectoriesStock_.swap(slqPtr->sTrajectoryStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeMissingSlqData() {

	// calculate state time derivative
	calculateFlowMap(nominalFlowMapTrajectoriesStock_);

	const scalar_t learningRate = 0.0;

	// calculate costate
	calculateRolloutCostate(nominalTimeTrajectoriesStock_,
			nominalCostateTrajectoriesStock_, learningRate);

	// calculate Lagrangian
	lagrange_array_t lagrangeMultiplierFunctionsStock(numPartitions_);
	calculateInputConstraintLagrangian(lagrangeMultiplierFunctionsStock, learningRate);
	calculateRolloutLagrangeMultiplier(
			nominalTimeTrajectoriesStock_, nominalStateTrajectoriesStock_, lagrangeMultiplierFunctionsStock,
			nominalLagrangianTrajectoriesStock_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateFlowMap(
		state_vector_array2_t& nominalFlowMapTrajectoriesStock)  {

	nominalFlowMapTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive subsystems
		if (i<initActivePartition_ || i>finalActivePartition_) {
			nominalFlowMapTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N = nominalTimeTrajectoriesStock_[i].size();
		nominalFlowMapTrajectoriesStock[i].resize(N);

		// set controller
		systemDynamicsPtr_->setController(nominalControllersStock_[i]);
		// initialize subsystem
		systemDynamicsPtr_->initializeModel(*(logicRulesMachinePtr_), i, "SLQ");

		for (size_t k=0; k<N; k++) {
			systemDynamicsPtr_->computeFlowMap(nominalTimeTrajectoriesStock_[i][k],
					nominalStateTrajectoriesStock_[i][k], nominalInputTrajectoriesStock_[i][k],
					nominalFlowMapTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop
}

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutCostate(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& costateTrajectoriesStock,
		scalar_t learningRate /*= 0.0*/ )  {

	costateTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			costateTrajectoriesStock[i].clear();
			continue;
		}

		SmFunc_.reset();
		SmFunc_.setTimeStamp(&SsTimeTrajectoriesStock_[i]);
		SmFunc_.setData(&SmTrajectoriesStock_[i]);
		SvFunc_.reset();
		SvFunc_.setTimeStamp(&SsTimeTrajectoriesStock_[i]);
		SvFunc_.setData(&SvTrajectoriesStock_[i]);
		SveFunc_.reset();
		SveFunc_.setTimeStamp(&SsTimeTrajectoriesStock_[i]);
		SveFunc_.setData(&SveTrajectoriesStock_[i]);
		nominalStateFunc_.reset();
		nominalStateFunc_.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		nominalStateFunc_.setData(&nominalStateTrajectoriesStock_[i]);

		const size_t N = timeTrajectoriesStock[i].size();
		costateTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			const scalar_t& t = timeTrajectoriesStock[i][k];

			state_matrix_t Sm;
			SmFunc_.interpolate(t, Sm);
			size_t greatestLessTimeStampIndex = SmFunc_.getGreatestLessTimeStampIndex();
			state_vector_t Sv;
			SvFunc_.interpolate(t, Sv, greatestLessTimeStampIndex);
			state_vector_t Sve;
			SveFunc_.interpolate(t, Sve, greatestLessTimeStampIndex);

			state_vector_t nominalState;
			nominalStateFunc_.interpolate(t, nominalState);

			costateTrajectoriesStock[i][k] = Sve + Sv
					+ learningRate * Sm * (nominalStateTrajectoriesStock_[i][k]-nominalState);

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateInputConstraintLagrangian(
		lagrange_array_t& lagrangeMultiplierFunctionsStock,
		scalar_t learningRate /*= 0.0*/) {

	// functions for controller and Lagrange multiplier
	LinearInterpolation<state_vector_t,Eigen::aligned_allocator<state_vector_t>> xFunc;
	LinearInterpolation<state_input_matrix_t,Eigen::aligned_allocator<state_input_matrix_t>> BmFunc;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> PmFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t>> RvFunc;
	LinearInterpolation<input_matrix_t,Eigen::aligned_allocator<input_matrix_t>> RmFunc;
	LinearInterpolation<input_vector_t,Eigen::aligned_allocator<input_vector_t>> EvProjectedFunc;
	LinearInterpolation<input_state_matrix_t,Eigen::aligned_allocator<input_state_matrix_t>> CmProjectedFunc;
	LinearInterpolation<control_constraint1_matrix_t,Eigen::aligned_allocator<control_constraint1_matrix_t>> DmDagerFunc;

	lagrangeMultiplierFunctionsStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			lagrangeMultiplierFunctionsStock[i].clear();
			continue;
		}

		xFunc.reset();
		xFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		xFunc.setData(&nominalStateTrajectoriesStock_[i]);

		BmFunc.reset();
		BmFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		BmFunc.setData(&BmTrajectoriesStock_[i]);

		PmFunc.reset();
		PmFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		PmFunc.setData(&PmTrajectoriesStock_[i]);

		RvFunc.reset();
		RvFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		RvFunc.setData(&RvTrajectoriesStock_[i]);

		RmFunc.reset();
		RmFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		RmFunc.setData(&RmTrajectoriesStock_[i]);

		EvProjectedFunc.reset();
		EvProjectedFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		EvProjectedFunc.setData(&EvProjectedTrajectoriesStock_[i]);

		CmProjectedFunc.reset();
		CmProjectedFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		CmProjectedFunc.setData(&CmProjectedTrajectoriesStock_[i]);

		DmDagerFunc.reset();
		DmDagerFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		DmDagerFunc.setData(&DmDagerTrajectoriesStock_[i]);

		const size_t N = SsTimeTrajectoriesStock_[i].size();

		lagrangeMultiplierFunctionsStock[i].time_ = SsTimeTrajectoriesStock_[i];
		lagrangeMultiplierFunctionsStock[i].k_.resize(N);
		lagrangeMultiplierFunctionsStock[i].uff_.resize(N);
		lagrangeMultiplierFunctionsStock[i].deltaUff_.resize(N);

		for (size_t k=0; k<N; k++) {

			const scalar_t& time = SsTimeTrajectoriesStock_[i][k];
			size_t greatestLessTimeStampIndex;

			state_vector_t nominalState;
			xFunc.interpolate(time, nominalState);
			greatestLessTimeStampIndex = xFunc.getGreatestLessTimeStampIndex();

			state_input_matrix_t Bm;
			BmFunc.interpolate(time, Bm, greatestLessTimeStampIndex);
			input_state_matrix_t Pm;
			PmFunc.interpolate(time, Pm, greatestLessTimeStampIndex);
			input_vector_t Rv;
			RvFunc.interpolate(time, Rv, greatestLessTimeStampIndex);
			input_vector_t EvProjected;
			EvProjectedFunc.interpolate(time, EvProjected, greatestLessTimeStampIndex);
			input_state_matrix_t CmProjected;
			CmProjectedFunc.interpolate(time, CmProjected, greatestLessTimeStampIndex);
			input_matrix_t Rm;
			RmFunc.interpolate(time, Rm, greatestLessTimeStampIndex);
			control_constraint1_matrix_t DmDager;
			DmDagerFunc.interpolate(time, DmDager, greatestLessTimeStampIndex);

			const size_t& nc1 = nc1TrajectoriesStock_[i][greatestLessTimeStampIndex];

			const state_matrix_t& Sm  = SmTrajectoriesStock_[i][k];
			const state_vector_t& Sv  = SvTrajectoriesStock_[i][k];
			const state_vector_t& Sve = SveTrajectoriesStock_[i][k];

			dynamic_input_matrix_t DmDagerTransRm = DmDager.leftCols(nc1).transpose() * Rm;

			constraint1_state_matrix_t& K  = lagrangeMultiplierFunctionsStock[i].k_[k];
			constraint1_vector_t& uff      = lagrangeMultiplierFunctionsStock[i].uff_[k];
			constraint1_vector_t& deltaUff = lagrangeMultiplierFunctionsStock[i].deltaUff_[k];

			K.topRows(nc1) = learningRate * ( DmDagerTransRm*CmProjected
					- DmDager.leftCols(nc1).transpose()*(Pm + Bm.transpose()*Sm) );
			K.bottomRows(DIMENSIONS::MAX_CONSTRAINT1_DIM_-nc1).setZero();

			uff.head(nc1) = DmDagerTransRm*EvProjected
					- DmDager.leftCols(nc1).transpose()*(Rv + Bm.transpose()*(Sv+Sve))
					- K.topRows(nc1)*nominalState;
			uff.tail(DIMENSIONS::MAX_CONSTRAINT1_DIM_-nc1).setZero();

			deltaUff.setZero();

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
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateRolloutLagrangeMultiplier(
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& stateTrajectoriesStock,
		const lagrange_array_t& lagrangeMultiplierFunctionsStock,
		constraint1_vector_array2_t& lagrangeTrajectoriesStock)  {

	LinearInterpolation<constraint1_vector_t, Eigen::aligned_allocator<constraint1_vector_t> > vffFunc;
	LinearInterpolation<constraint1_state_matrix_t, Eigen::aligned_allocator<constraint1_state_matrix_t> > vfbFunc;

	lagrangeTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip the inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			lagrangeTrajectoriesStock[i].clear();
			continue;
		}

		vffFunc.reset();
		vffFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vffFunc.setData(&lagrangeMultiplierFunctionsStock[i].uff_);

		vfbFunc.reset();
		vfbFunc.setTimeStamp(&lagrangeMultiplierFunctionsStock[i].time_);
		vfbFunc.setData(&lagrangeMultiplierFunctionsStock[i].k_);

		size_t N = timeTrajectoriesStock[i].size();
		lagrangeTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			constraint1_vector_t vff;
			vffFunc.interpolate(timeTrajectoriesStock[i][k], vff);
			size_t greatestLessTimeIndex = vffFunc.getGreatestLessTimeStampIndex();

			constraint1_state_matrix_t vfb;
			vfbFunc.interpolate(timeTrajectoriesStock[i][k], vfb, greatestLessTimeIndex);

			lagrangeTrajectoriesStock[i][k] = vff + vfb*stateTrajectoriesStock[i][k];

		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findActiveSubsystemIndex(
		const scalar_array_t& eventTimes,
		const scalar_t& time,
		bool ceilingFunction /*= true*/) const {

	scalar_array_t partitioningTimes(eventTimes.size()+2);
	partitioningTimes.front() = std::numeric_limits<scalar_t>::lowest();
	partitioningTimes.back()  = std::numeric_limits<scalar_t>::max();
	for (size_t i=0; i<eventTimes.size(); i++)
		partitioningTimes[i+1] = eventTimes[i];

	int activeSubsystemIndex;
	if (ceilingFunction==true)
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);
	else
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0,
				-OCS2NumericTraits<scalar_t>::week_epsilon());

	return (size_t)activeSubsystemIndex;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
size_t GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::findActivePartitionIndex(
		const scalar_array_t& partitioningTimes,
		const scalar_t& time,
		bool ceilingFunction /*= true*/) const {

	int activeSubsystemIndex;
	if (ceilingFunction==true)
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0);
	else
		activeSubsystemIndex = findActiveIntervalIndex(partitioningTimes, time, 0,
				-OCS2NumericTraits<scalar_t>::week_epsilon());

	if (activeSubsystemIndex < 0) {
		std::string mesg = "Given time is less than the start time (i.e. givenTime < partitioningTimes.front()): "
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

/*****************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::computeEquivalentSystemMultiplier(
		const size_t& eventTimeIndex,
		const size_t& activeSubsystem,
		scalar_t& multiplier) const {

	if (activeSubsystem == eventTimeIndex+1) {
		if (activeSubsystem+1 == eventTimes().size()) {
			if (finalTime_<eventTimes()[eventTimeIndex])
				throw std::runtime_error("final time is smaller than the last triggered event time.");
			else
				multiplier = -1.0 / (finalTime_ - eventTimes()[eventTimeIndex]);
		} else {
			multiplier = -1.0 / (eventTimes()[eventTimeIndex+1] - eventTimes()[eventTimeIndex]);
		}

	} else if (activeSubsystem == eventTimeIndex)
		if (activeSubsystem == 0) {
			if (finalTime_<eventTimes()[eventTimeIndex])
				throw std::runtime_error("Initial time is greater than the last triggered event time.");
			else
				multiplier = 1.0 / (eventTimes()[eventTimeIndex] - initTime_);
		} else {
			multiplier = 1.0 / (eventTimes()[eventTimeIndex] - eventTimes()[eventTimeIndex-1]);
		}
	else
		multiplier = 0.0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getRolloutSensitivity2SwitchingTime(
		const size_t& eventTimeIndex,
		std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		state_matrix_array2_t& sensitivityStateTrajectoriesStock,
		input_matrix_array2_t& sensitivityInputTrajectoriesStock)  {

	if (eventTimeIndex+1 > numEventTimes_)
		throw std::runtime_error("The requested event index is out of bound.");

	sensitivityTimeTrajectoriesStock  = nominalTimeTrajectoriesStock_;
	sensitivityStateTrajectoriesStock = sensitivityStateTrajectoriesStockSet_[eventTimeIndex];
	sensitivityInputTrajectoriesStock = sensitivityInputTrajectoriesStockSet_[eventTimeIndex];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getController(
		controller_array_t& controllersStock) {

	slqPtr_->getController(controllersStock);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getValueFuntion(
		const scalar_t& time,
		const state_vector_t& state,
		scalar_t& valueFuntion)  {

	slqPtr_->getValueFuntion(time, state, valueFuntion);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostFuntion(
		scalar_t& costFunction,
		scalar_t& constraintISE)  {

	slqPtr_->getCostFuntion(costFunction, constraintISE);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
template <typename Derived>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getCostFuntionDerivative(
		Eigen::MatrixBase<Derived> const& costFunctionDerivative) const {

	// refer to Eigen documentation under the topic "Writing Functions Taking Eigen Types as Parameters"
	const_cast<Eigen::MatrixBase<Derived>&>(costFunctionDerivative) = nominalCostFuntionDerivative_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getNominalTrajectories(
		std::vector<scalar_array_t>& nominalTimeTrajectoriesStock,
		state_vector_array2_t& nominalStateTrajectoriesStock,
		input_vector_array2_t& nominalInputTrajectoriesStock)   {

	slqPtr_->getNominalTrajectories(nominalTimeTrajectoriesStock, nominalStateTrajectoriesStock, nominalInputTrajectoriesStock);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
const typename GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::scalar_array_t&
	GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::eventTimes() const {

	return logicRulesMachinePtr_->getLogicRulesPtr()->eventTimes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::propagateRolloutSensitivity(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const controller_array_t& controllersStock,
		const input_vector_array2_t& LvTrajectoriesStock,
		const std::vector<scalar_array_t>& sensitivityTimeTrajectoriesStock,
		const std::vector<size_array_t>& eventsPastTheEndIndecesStock,
		state_vector_array2_t& sensitivityStateTrajectoriesStock,
		input_vector_array2_t& sensitivityInputTrajectoriesStock)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	sensitivityStateTrajectoriesStock.resize(numPartitions_);
	sensitivityInputTrajectoriesStock.resize(numPartitions_);

	// Initial state sensitivity (which is zero)
	state_vector_t nabla_xInit = state_vector_t::Zero();

	for (size_t i=0; i<numPartitions_; i++) {

		// skip inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			sensitivityStateTrajectoriesStock[i].clear();
			sensitivityInputTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N  = sensitivityTimeTrajectoriesStock[i].size();
		const size_t NE = eventsPastTheEndIndecesStock[i].size();

		// set data for rollout sensitivity equation
		rolloutSensitivityEquationsPtrStock_[workerIndex]->reset();
		rolloutSensitivityEquationsPtrStock_[workerIndex]->setData(
				&sensitivityTimeTrajectoriesStock[i],
				&AmTrajectoriesStock_[i], &BmTrajectoriesStock_[i], &nominalFlowMapTrajectoriesStock_[i],
				&controllersStock[i].time_,
				&LvTrajectoriesStock[i], &controllersStock[i].k_);

		// max number of steps of integration
		const size_t maxNumSteps = settingsPtr_->maxNumStepsPerSecond_ *
				std::max(1.0, sensitivityTimeTrajectoriesStock[i].back()-sensitivityTimeTrajectoriesStock[i].front());

		// resizing
		sensitivityStateTrajectoriesStock[i].clear();
		sensitivityStateTrajectoriesStock[i].reserve(N);
		sensitivityInputTrajectoriesStock[i].clear();
		sensitivityInputTrajectoriesStock[i].reserve(N);

		// integrating
		size_t k_u = 0;
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = (j==0) ? sensitivityTimeTrajectoriesStock[i].begin()
					: sensitivityTimeTrajectoriesStock[i].begin() + eventsPastTheEndIndecesStock[i][j-1];
			endTimeItr = (j==NE) ? sensitivityTimeTrajectoriesStock[i].end()
					: sensitivityTimeTrajectoriesStock[i].begin() + eventsPastTheEndIndecesStock[i][j];

			// if it should be integrated
			if (endTimeItr != beginTimeItr) {

				// finding the current active subsystem
				scalar_t midTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				size_t activeSubsystem = findActiveSubsystemIndex(
						eventTimes(), midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				rolloutSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve sensitivity ODE
				rolloutSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						nabla_xInit, beginTimeItr, endTimeItr,
						sensitivityStateTrajectoriesStock[i],
						settingsPtr_->minTimeStep_, settingsPtr_->absTolODE_, settingsPtr_->relTolODE_);

				// compute input sensitivity
				for ( ; k_u<sensitivityStateTrajectoriesStock[i].size(); k_u++) {
					sensitivityInputTrajectoriesStock[i].emplace_back(
							rolloutSensitivityEquationsPtrStock_[workerIndex]->computeInput(
									sensitivityTimeTrajectoriesStock[i][k_u], sensitivityStateTrajectoriesStock[i][k_u]) );
				} // end of k loop
			}

			// compute jump map
			if (j < NE) {
				nabla_xInit =  sensitivityStateTrajectoriesStock[i].back();
			}

		}  // end of j loop

		// reset the initial state
		nabla_xInit = sensitivityStateTrajectoriesStock[i].back();

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::approximateNominalLQPSensitivity2SwitchingTime(
		const state_vector_array2_t& sensitivityStateTrajectoriesStock,
		const input_vector_array2_t& sensitivityInputTrajectoriesStock,
		eigen_scalar_array2_t& nablaqTrajectoriesStock,
		state_vector_array2_t& nablaQvTrajectoriesStock,
		input_vector_array2_t& nablaRvTrajectoriesStock,
		eigen_scalar_array2_t& nablaqFinalStock,
		state_vector_array2_t& nablaQvFinalStock) const {

	// resizing
	nablaqTrajectoriesStock.resize(numPartitions_);
	nablaQvTrajectoriesStock.resize(numPartitions_);
	nablaRvTrajectoriesStock.resize(numPartitions_);
	nablaqFinalStock.resize(numPartitions_);
	nablaQvFinalStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		// skip inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			nablaqTrajectoriesStock[i].clear();
			nablaQvTrajectoriesStock[i].clear();
			nablaRvTrajectoriesStock[i].clear();
			nablaqFinalStock[i].clear();
			nablaQvFinalStock[i].clear();
			continue;
		}

		const size_t N  = nominalTimeTrajectoriesStock_[i].size();
		const size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();
		auto eventsPastTheEndItr = nominalEventsPastTheEndIndecesStock_[i].begin();

		// resizing
		nablaqTrajectoriesStock[i].resize(N);
		nablaQvTrajectoriesStock[i].resize(N);
		nablaRvTrajectoriesStock[i].resize(N);
		nablaqFinalStock[i].resize(NE);
		nablaQvFinalStock[i].resize(NE);

		for (size_t k=0; k<N; k++) {

			const input_matrix_t& Rm = RmTrajectoriesStock_[i][k];
			const state_vector_t& Qv = QvTrajectoriesStock_[i][k];
			const state_matrix_t& Qm = QmTrajectoriesStock_[i][k];
			const input_vector_t& Rv = RvTrajectoriesStock_[i][k];
			const input_state_matrix_t& Pm = PmTrajectoriesStock_[i][k];

			nablaqTrajectoriesStock[i][k]  = Qv.transpose()*sensitivityStateTrajectoriesStock[i][k] +
					Rv.transpose()*sensitivityInputTrajectoriesStock[i][k];
			nablaQvTrajectoriesStock[i][k] = Qm*sensitivityStateTrajectoriesStock[i][k] +
					Pm.transpose()*sensitivityInputTrajectoriesStock[i][k];
			nablaRvTrajectoriesStock[i][k] = Pm*sensitivityStateTrajectoriesStock[i][k] +
					Rm*sensitivityInputTrajectoriesStock[i][k];

			// terminal cost sensitivity to switching times
			if (eventsPastTheEndItr!=nominalEventsPastTheEndIndecesStock_[i].end() && k+1==*eventsPastTheEndItr) {

				const size_t eventIndex = eventsPastTheEndItr - nominalEventsPastTheEndIndecesStock_[i].begin();
				const size_t timeIndex = *eventsPastTheEndItr - 1;
				const state_vector_t& Qv = QvFinalStock_[i][eventIndex];
				const state_matrix_t& Qm = QmFinalStock_[i][eventIndex];

				nablaqFinalStock[i][eventIndex]  = Qv.transpose()*sensitivityStateTrajectoriesStock[i][timeIndex];
				nablaQvFinalStock[i][eventIndex] = Qm*sensitivityStateTrajectoriesStock[i][timeIndex];

				eventsPastTheEndItr++;
			}
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSensitivityRiccatiEquations(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const scalar_t& learningRate,
		eigen_scalar_array2_t& nablasTrajectoriesStock,
		state_vector_array2_t& nablaSvTrajectoriesStock,
		state_matrix_array2_t& nablaSmTrajectoriesStock)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	typedef typename riccati_sensitivity_equations_t::s_vector_t s_vector_t;
	typedef typename riccati_sensitivity_equations_t::s_vector_array_t s_vector_array_t;

	// Resizing
	nablasTrajectoriesStock.resize(numPartitions_);
	nablaSvTrajectoriesStock.resize(numPartitions_);
	nablaSmTrajectoriesStock.resize(numPartitions_);

	// temporal final value for the last Riccati equations
	s_vector_t SsFinalTemp = s_vector_t::Zero();
	// output containers which is reverse
	s_vector_array_t allSsTrajectory;

	for (int i=numSubsystems_-1; i>=0; i--) {

		// skip inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			nablasTrajectoriesStock[i].clear();
			nablaSvTrajectoriesStock[i].clear();
			nablaSmTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N  = SsNormalizedTimeTrajectoriesStock_[i].size();
		const size_t NE = SsNormalizedEventsPastTheEndIndecesStock_[i].size();

		// set data for Riccati sensitivity equations
		riccatiSensitivityEquationsPtrStock_[workerIndex]->reset();
		riccatiSensitivityEquationsPtrStock_[workerIndex]->setData(
				learningRate, partitioningTimes_[i], partitioningTimes_[i+1],
				&SsTimeTrajectoriesStock_[i],
				&SmTrajectoriesStock_[i],
				&SvTrajectoriesStock_[i],
				&nominalTimeTrajectoriesStock_[i],
				&AmTrajectoriesStock_[i],
				&BmTrajectoriesStock_[i],
				&qTrajectoriesStock_[i],
				&QvTrajectoriesStock_[i],
				&QmTrajectoriesStock_[i],
				&RvTrajectoriesStock_[i],
				&RmInverseTrajectoriesStock_[i],
				&RmTrajectoriesStock_[i],
				&PmTrajectoriesStock_[i],
				&nablaqTrajectoriesStockSet_[eventTimeIndex][i],
				&nablaQvTrajectoriesStockSet_[eventTimeIndex][i],
				&nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

		// max number of steps of integration
		const size_t maxNumSteps = settingsPtr_->maxNumStepsPerSecond_ *
				std::max(1.0, SsNormalizedTimeTrajectoriesStock_[i].back()-SsNormalizedTimeTrajectoriesStock_[i].front());

		// output containers resizing
		nablasTrajectoriesStock[i].resize(N);
		nablaSvTrajectoriesStock[i].resize(N);
		nablaSmTrajectoriesStock[i].resize(N);
		allSsTrajectory.clear();
		allSsTrajectory.reserve(N);

		// integrating the Riccati sensitivity equations
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = (j==0) ? SsNormalizedTimeTrajectoriesStock_[i].begin()
					: SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedEventsPastTheEndIndecesStock_[i][j-1];
			endTimeItr = (j==NE) ? SsNormalizedTimeTrajectoriesStock_[i].end()
					: SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedEventsPastTheEndIndecesStock_[i][j];

			// if the event time does not take place at the end of partition
			if (endTimeItr != beginTimeItr) {

				// finding the current active subsystem
				scalar_t midNormalizedTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				scalar_t midTime = partitioningTimes_[i+1] - (partitioningTimes_[i+1]-partitioningTimes_[i])*midNormalizedTime;
				size_t activeSubsystem = findActiveSubsystemIndex(
						eventTimes(), midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				riccatiSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve Riccati sensitivity equations
				riccatiSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						SsFinalTemp, beginTimeItr, endTimeItr,
						allSsTrajectory,
						settingsPtr_->minTimeStep_, settingsPtr_->absTolODE_, settingsPtr_->relTolODE_, maxNumSteps, true);
			}

			if (j < NE) {
				SsFinalTemp = allSsTrajectory.back();
			}

		}  // end of j loop

		// final value of the next partition
		SsFinalTemp = allSsTrajectory.back();

		// construct 'nable_Sm', 'nable_Sv', and 'nable_s'
		for (size_t k=0; k<N; k++) {
			riccati_sensitivity_equations_t::convert2Matrix(allSsTrajectory[N-1-k],
					nablaSmTrajectoriesStock[i][k], nablaSvTrajectoriesStock[i][k], nablasTrajectoriesStock[i][k]);
		}  // end of k loop

	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::solveSensitivityBVP(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const state_vector_t& MvFinal,
		state_vector_array2_t& MvTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// Resizing
	MvTrajectoriesStock.resize(numPartitions_);

	// temporal final value for the last Riccati equations
	state_vector_t MvFinalTemp = MvFinal;
	// output containers which is reverse
	state_vector_array_t rMvTrajectory;

	for (int i=numPartitions_-1; i>=0; i--) {

		// skip inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			MvTrajectoriesStock[i].clear();
			continue;
		}

		const size_t N  = SsNormalizedTimeTrajectoriesStock_[i].size();
		const size_t NE = SsNormalizedEventsPastTheEndIndecesStock_[i].size();

		// set data for Riccati equations
		bvpSensitivityEquationsPtrStock_[workerIndex]->reset();
		bvpSensitivityEquationsPtrStock_[workerIndex]->resetNumFunctionCalls();
		bvpSensitivityEquationsPtrStock_[workerIndex]->setData(
				partitioningTimes_[i], partitioningTimes_[i+1],
				&nominalTimeTrajectoriesStock_[i],
				&AmTrajectoriesStock_[i], &BmTrajectoriesStock_[i], &CmTrajectoriesStock_[i],
				&AmConstrainedTrajectoriesStock_[i], &CmTrajectoriesStock_[i],
				&QvTrajectoriesStock_[i], &RmTrajectoriesStock_[i],
				&nominalFlowMapTrajectoriesStock_[i], &nominalCostateTrajectoriesStock_[i], &nominalLagrangianTrajectoriesStock_[i],
				&nominalControllersStock_[i].time_, &nominalControllersStock_[i].k_, &SmTrajectoriesStock_[i]);

		// max number of steps of integration
		const size_t maxNumSteps = settingsPtr_->maxNumStepsPerSecond_ *
				std::max(1.0, SsNormalizedTimeTrajectoriesStock_[i].back()-SsNormalizedTimeTrajectoriesStock_[i].front());

		// output containers resizing
		MvTrajectoriesStock[i].resize(N);
		rMvTrajectory.clear();
		rMvTrajectory.reserve(N);

		// integrating the Riccati equations
		typename scalar_array_t::const_iterator beginTimeItr, endTimeItr;
		for (size_t j=0; j<=NE; j++) {

			beginTimeItr = (j==0) ? SsNormalizedTimeTrajectoriesStock_[i].begin()
					: SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedEventsPastTheEndIndecesStock_[i][j-1];
			endTimeItr = (j==NE) ? SsNormalizedTimeTrajectoriesStock_[i].end()
					: SsNormalizedTimeTrajectoriesStock_[i].begin() + SsNormalizedEventsPastTheEndIndecesStock_[i][j];

			// if the event time does not take place at the end of partition
			if (endTimeItr != beginTimeItr) {

				// finding the current active subsystem
				scalar_t midNormalizedTime = 0.5 * (*beginTimeItr+*(endTimeItr-1));
				scalar_t midTime = partitioningTimes_[i+1] - (partitioningTimes_[i+1]-partitioningTimes_[i])*midNormalizedTime;
				size_t activeSubsystem = findActiveSubsystemIndex(
						eventTimes(), midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);
				bvpSensitivityEquationsPtrStock_[workerIndex]->setMultiplier(multiplier);

				// solve Riccati equations
				bvpSensitivityIntegratorsPtrStock_[workerIndex]->integrate(
						MvFinalTemp, beginTimeItr, endTimeItr,
						rMvTrajectory,
						settingsPtr_->minTimeStep_, settingsPtr_->absTolODE_, settingsPtr_->relTolODE_, maxNumSteps, true);
			}

			if (j < NE) {
				bvpSensitivityEquationsPtrStock_[workerIndex]->computeJumpMap(*endTimeItr, rMvTrajectory.back(),
						MvFinalTemp);
			}

		}  // end of j loop

		// final value of the next partition
		MvFinalTemp = rMvTrajectory.back();

		// constructing 'Mv'
		for (size_t k=0; k<N; k++)
			MvTrajectoriesStock[i][k] = rMvTrajectory[N-1-k];

		// testing the numerical stability of the Riccati equations
		if (settingsPtr_->checkNumericalStability_)
			for (int k=N-1; k>=0; k--) {
				try {
					if (MvTrajectoriesStock[i][k].hasNaN())
						throw std::runtime_error("Mv is unstable.");
				}
				catch(const std::exception& error)
				{
					std::cerr << "what(): " << error.what() << " at time " << SsTimeTrajectoriesStock_[i][k] << " [sec]." << std::endl;
					for (int kp=k; kp<k+10; kp++)  {
						if (kp >= N) continue;
						std::cerr << "Mv[" << SsTimeTrajectoriesStock_[i][kp] << "]:\t"<< MvTrajectoriesStock[i][kp].transpose().norm() << std::endl;
					}
					exit(0);
				}
			}

	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateLQSensitivityControllerForward(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& nablaSvTrajectoriesStock,
		input_vector_array2_t& nablaLvTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	nablaLvTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numSubsystems_; i++) {

		// skip inactive partitions
		if (i<initActivePartition_ || i>finalActivePartition_) {
			nablaLvTrajectoriesStock[i].clear();
			continue;
		}

		// set data
		BmFuncStock_[workerIndex].reset();
		BmFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		BmFuncStock_[workerIndex].setData(&BmTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].reset();
		RmInverseFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].setData(&RmInverseTrajectoriesStock_[i]);
		nablaRvFuncStock_[workerIndex].reset();
		nablaRvFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		nablaRvFuncStock_[workerIndex].setData(&nablaRvTrajectoriesStockSet_[eventTimeIndex][i]);

		// resizing
		const size_t N = nablaSvTrajectoriesStock[i].size();
		nablaLvTrajectoriesStock[i].resize(N);

		for (size_t k=0; k<N; k++) {

			// time
			const scalar_t& t = timeTrajectoriesStock[i][k];

			// Bm
			state_input_matrix_t Bm;
			BmFuncStock_[workerIndex].interpolate(t, Bm);
			size_t greatestLessTimeStampIndex = BmFuncStock_[workerIndex].getGreatestLessTimeStampIndex();
			// RmInverse
			input_matrix_t RmInverse;
			RmInverseFuncStock_[workerIndex].interpolate(t, RmInverse, greatestLessTimeStampIndex);
			// nablaRv
			input_vector_t nablaRv;
			nablaRvFuncStock_[workerIndex].interpolate(t, nablaRv, greatestLessTimeStampIndex);

			nablaLvTrajectoriesStock[i][k] = -RmInverse * (nablaRv + Bm.transpose()*nablaSvTrajectoriesStock[i][k]);
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateBVPSensitivityControllerForward(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const std::vector<scalar_array_t>& timeTrajectoriesStock,
		const state_vector_array2_t& MvTrajectoriesStock,
		input_vector_array2_t& LvTrajectoriesStock) {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	// resizing
	LvTrajectoriesStock.resize(numPartitions_);

	for (size_t i=0; i<numPartitions_; i++) {

		if (i<initActivePartition_ || i>finalActivePartition_) {
			LvTrajectoriesStock[i].clear();
			continue;
		}

		// set data
		BmFuncStock_[workerIndex].reset();
		BmFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		BmFuncStock_[workerIndex].setData(&BmTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].reset();
		RmInverseFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		RmInverseFuncStock_[workerIndex].setData(&RmInverseTrajectoriesStock_[i]);
		DmProjectedFuncStock_[workerIndex].reset();
		DmProjectedFuncStock_[workerIndex].setTimeStamp(&nominalTimeTrajectoriesStock_[i]);
		DmProjectedFuncStock_[workerIndex].setData(&DmProjectedTrajectoriesStock_[i]);

		const size_t N = timeTrajectoriesStock[i].size();
		LvTrajectoriesStock[i].resize(N);
		for (size_t k=0; k<N; k++) {

			// time
			const scalar_t& t = timeTrajectoriesStock[i][k];
			// Bm
			state_input_matrix_t Bm;
			BmFuncStock_[workerIndex].interpolate(t, Bm);
			size_t greatestLessTimeStampIndex = BmFuncStock_[workerIndex].getGreatestLessTimeStampIndex();
			// RmInverse
			input_matrix_t RmInverse;
			RmInverseFuncStock_[workerIndex].interpolate(t, RmInverse, greatestLessTimeStampIndex);
			// DmProjected
			input_matrix_t DmProjected;
			DmProjectedFuncStock_[workerIndex].interpolate(t, DmProjected, greatestLessTimeStampIndex);

			LvTrajectoriesStock[i][k] = -(input_matrix_t::Identity()-DmProjected)
					* RmInverse * Bm.transpose() * MvTrajectoriesStock[i][k];
		}  // end of k loop
	}  // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::getValueFuntionDerivative(
		const size_t& eventTimeIndex,
		const scalar_t& time,
		const state_vector_t& state,
		scalar_t& valueFunctionDerivative)  {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	EigenLinearInterpolation<state_vector_t> nominalStateFunc;
	EigenLinearInterpolation<eigen_scalar_t> nablasFunc;
	EigenLinearInterpolation<state_vector_t> nablaSvFunc;
	EigenLinearInterpolation<state_matrix_t> nablaSmFunc;

	size_t activePartition = findActivePartitionIndex(partitioningTimes_, time);

	state_vector_t nominalState;
	state_vector_t deltsState;
	eigen_scalar_t nablas;
	state_vector_t nablaSv;
	state_matrix_t nablaSm;
	size_t greatestLessTimeStampIndex = 0;

	nominalStateFunc.reset();
	nominalStateFunc.setTimeStamp(&nominalTimeTrajectoriesStock_[activePartition]);
	nominalStateFunc.setData(&nominalStateTrajectoriesStock_[activePartition]);
	nominalStateFunc.interpolate(time, nominalState);
	deltsState = state - nominalState;

	nablasFunc.reset();
	nablasFunc.setTimeStamp(&SsTimeTrajectoriesStock_[activePartition]);
	nablasFunc.setData(&nablasTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	nablasFunc.interpolate(time, nablas);
	greatestLessTimeStampIndex = nablasFunc.getGreatestLessTimeStampIndex();

	nablaSvFunc.reset();
	nablaSvFunc.setTimeStamp(&SsTimeTrajectoriesStock_[activePartition]);
	nablaSvFunc.setData(&nablaSvTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	nablaSvFunc.interpolate(time, nablaSv, greatestLessTimeStampIndex);

	nablaSmFunc.reset();
	nablaSmFunc.setTimeStamp(&SsTimeTrajectoriesStock_[activePartition]);
	nablaSmFunc.setData(&nablaSmTrajectoriesStockSet_[eventTimeIndex][activePartition]);
	nablaSmFunc.interpolate(time, nablaSm, greatestLessTimeStampIndex);

	valueFunctionDerivative = nablas(0) +
			deltsState.dot(nablaSv) + 0.5*deltsState.dot(nablaSm*deltsState);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::calculateCostDerivative(
		size_t workerIndex,
		const size_t& eventTimeIndex,
		const state_vector_array2_t& sensitivityStateTrajectoriesStock,
		const input_vector_array2_t& sensitivityInputTrajectoriesStock,
		scalar_t& costDerivative) const {

	if (eventTimeIndex<activeEventTimeBeginIndex_ || eventTimeIndex>=activeEventTimeEndIndex_)
		throw std::runtime_error("The index is associated to an inactive event or it is out of range.");

	costDerivative = 0.0;
	scalar_t prevIntermediatecostDev = 0.0;
	scalar_t currIntermediatecostDev = 0.0;
	size_t beginIndex, endIndex;

	for (size_t i=initActivePartition_; i<=finalActivePartition_; i++) {

		const size_t N  = nominalTimeTrajectoriesStock_[i].size();
		const size_t NE = nominalEventsPastTheEndIndecesStock_[i].size();

		for (size_t j=0; j<NE+1; j++) {

			beginIndex = (j==0)  ? 0 : nominalEventsPastTheEndIndecesStock_[i][j-1];
			endIndex   = (j==NE) ? N : nominalEventsPastTheEndIndecesStock_[i][j];

			// integrates the intermediate cost sensitivity using the trapezoidal approximation method
			if (beginIndex != endIndex) {

				// finding the current active subsystem
				scalar_t midTime = 0.5 * ( nominalTimeTrajectoriesStock_[i][beginIndex] +
						nominalTimeTrajectoriesStock_[i][endIndex-1]);
				size_t activeSubsystem = findActiveSubsystemIndex(
						eventTimes(), midTime);

				// compute multiplier of the equivalent system
				scalar_t multiplier;
				computeEquivalentSystemMultiplier(eventTimeIndex, activeSubsystem, multiplier);

				for (size_t k=beginIndex; k<endIndex; k++) {

					if (k>beginIndex)
						prevIntermediatecostDev = currIntermediatecostDev;

					currIntermediatecostDev =
							multiplier * qTrajectoriesStock_[i][k](0) +
							sensitivityStateTrajectoriesStock[i][k].dot(QvTrajectoriesStock_[i][k]) +
							sensitivityInputTrajectoriesStock[i][k].dot(RvTrajectoriesStock_[i][k]);

					if (k>beginIndex) {
						costDerivative += 0.5*(nominalTimeTrajectoriesStock_[i][k]-nominalTimeTrajectoriesStock_[i][k-1]) *
								(currIntermediatecostDev+prevIntermediatecostDev);
					}
				}  // end of k loop
			}

			// terminal cost sensitivity at switching times
			if (j < NE) {
				costDerivative += sensitivityStateTrajectoriesStock[i].back().transpose() * QvFinalStock_[i][j];
			}
		}  // end of j loop


	}  // end of i loop

	// TODO: add heuristic to SLQ
//	throw std::runtime_error("The Heuristic cost evaluation is missing.");
	// calculate the Heuristics function at the final time
//	// initialize
//	heuristicsFunctionsPtrStock_[threadId]->initializeModel(*logicRulesMachinePtr_, finalActivePartition_, "SLQ");
//	// set desired trajectories
//	heuristicsFunctionsPtrStock_[threadId]->setCostDesiredTrajectories(costDesiredTrajectories_);
//	// set state-input
//	heuristicsFunctionsPtrStock_[threadId]->setCurrentStateAndControl(
//			timeTrajectoriesStock[finalActivePartition_].back(),
//			stateTrajectoriesStock[finalActivePartition_].back(),
//			inputTrajectoriesStock[finalActivePartition_].back());
//	// compute
//	scalar_t sHeuristics;
//	heuristicsFunctionsPtrStock_[threadId]->getTerminalCost(sHeuristics);
//	totalCost += sHeuristics;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runLQBasedMethod()  {

	const size_t maxNumIteration = 3;

	// resizing
	nablaLvTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
	nablaqTrajectoriesStockSet_.resize(numEventTimes_);
	nablaQvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaRvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaqFinalStockSet_.resize(numEventTimes_);
	nablaQvFinalStockSet_.resize(numEventTimes_);
	nablasTrajectoriesStockSet_.resize(numEventTimes_);
	nablaSvTrajectoriesStockSet_.resize(numEventTimes_);
	nablaSmTrajectoriesStockSet_.resize(numEventTimes_);
	nominalCostFuntionDerivative_.resize(numEventTimes_);

	size_t iteration = 0;
	while (iteration++ < maxNumIteration) {

		// for each active event time
		for (size_t index=0; index<numEventTimes_; index++) {

			if (activeEventTimeBeginIndex_<=index && index<activeEventTimeEndIndex_) {

				// for the first iteration set Lv to zero
				if (iteration == 1) {
					nablaLvTrajectoriesStockSet_[index].resize(numPartitions_);
					for (size_t i=initActivePartition_; i<=finalActivePartition_; i++)
						nablaLvTrajectoriesStockSet_[index][i] = input_vector_array_t(
								nominalControllersStock_[i].time_.size(), input_vector_t::Zero());
				}

				const size_t workerIndex = 0;

				// calculate rollout sensitivity to event times
				propagateRolloutSensitivity(workerIndex, index,
						nominalControllersStock_,
						nablaLvTrajectoriesStockSet_[index],
						nominalTimeTrajectoriesStock_,
						nominalEventsPastTheEndIndecesStock_,
						sensitivityStateTrajectoriesStockSet_[index],
						sensitivityInputTrajectoriesStockSet_[index]);

				// approximate the nominal LQ sensitivity to switching times
				approximateNominalLQPSensitivity2SwitchingTime(
						sensitivityStateTrajectoriesStockSet_[index], sensitivityInputTrajectoriesStockSet_[index],
						nablaqTrajectoriesStockSet_[index],
						nablaQvTrajectoriesStockSet_[index], nablaRvTrajectoriesStockSet_[index],
						nablaqFinalStockSet_[index], nablaQvFinalStockSet_[index]);

				// solve Riccati equations
				// prevents the changes in the nominal trajectories and just update the gains
				const scalar_t learningRateStar = 0.0;
				solveSensitivityRiccatiEquations(workerIndex, index,
						learningRateStar,
						nablasTrajectoriesStockSet_[index],
						nablaSvTrajectoriesStockSet_[index],
						nablaSmTrajectoriesStockSet_[index]);

				// calculate sensitivity controller feedforward part
				calculateLQSensitivityControllerForward(workerIndex, index,
						SsTimeTrajectoriesStock_, nablaSvTrajectoriesStockSet_[index],
						nablaLvTrajectoriesStockSet_[index]);

				// calculate the value function derivatives w.r.t. switchingTimes
				getValueFuntionDerivative(index, initTime_, initState_,
						nominalCostFuntionDerivative_(index));

			} else if (iteration == 1) {
				nablaLvTrajectoriesStockSet_[index].clear();
				sensitivityStateTrajectoriesStockSet_[index].clear();
				sensitivityInputTrajectoriesStockSet_[index].clear();
				nablaqTrajectoriesStockSet_[index].clear();
				nablaQvTrajectoriesStockSet_[index].clear();
				nablaRvTrajectoriesStockSet_[index].clear();
				nablaqFinalStockSet_[index].clear();
				nablaQvFinalStockSet_[index].clear();
				nablasTrajectoriesStockSet_[index].clear();
				nablaSvTrajectoriesStockSet_[index].clear();
				nablaSmTrajectoriesStockSet_[index].clear();
				nominalCostFuntionDerivative_(index) = 0.0;
			}

		}  // end of index loop

	}  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::runSweepingBVPMethod()  {

	// resizing
	MvTrajectoriesStockSet_.resize(numEventTimes_);
	LvTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityStateTrajectoriesStockSet_.resize(numEventTimes_);
	sensitivityInputTrajectoriesStockSet_.resize(numEventTimes_);
	nominalCostFuntionDerivative_.resize(numEventTimes_);

	// for each active event time
	for (size_t index=0; index<numEventTimes_; index++) {

		if (activeEventTimeBeginIndex_<=index && index<activeEventTimeEndIndex_) {

			const size_t workerIndex = 0;

			// solve BVP to compute Mv
			solveSensitivityBVP(workerIndex, index, state_vector_t::Zero(),
					MvTrajectoriesStockSet_[index]);

			// calculates sensitivity controller feedforward part, Lv
			calculateBVPSensitivityControllerForward(workerIndex, index,
					SsTimeTrajectoriesStock_, MvTrajectoriesStockSet_[index],
					LvTrajectoriesStockSet_[index]);

			// calculate rollout sensitivity to event times
			propagateRolloutSensitivity(workerIndex, index,
					nominalControllersStock_,
					LvTrajectoriesStockSet_[index],
					nominalTimeTrajectoriesStock_,
					nominalEventsPastTheEndIndecesStock_,
					sensitivityStateTrajectoriesStockSet_[index],
					sensitivityInputTrajectoriesStockSet_[index]);

			// calculate the cost function derivatives w.r.t. switchingTimes
			calculateCostDerivative(workerIndex, index,
					sensitivityStateTrajectoriesStockSet_[index],
					sensitivityInputTrajectoriesStockSet_[index],
					nominalCostFuntionDerivative_(index));

		} else {
			MvTrajectoriesStockSet_[index].clear();
			LvTrajectoriesStockSet_[index].clear();
			sensitivityStateTrajectoriesStockSet_[index].clear();
			sensitivityInputTrajectoriesStockSet_[index].clear();
			nominalCostFuntionDerivative_(index) = 0.0;
		}

	} // end of index
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void GSLQ<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::run()  {

	// update numPartitions_ if it has been changed
	if (numPartitions_ != slqPtr_->numPartitions_) {
		numPartitions_ = slqPtr_->numPartitions_;
		setupOptimizer(numPartitions_);
	}

	// collect or compute required data from the SLQ instance
	collectSlqData(slqPtr_);
	computeMissingSlqData();

	// number of event and subsystems
	numEventTimes_ = eventTimes().size();
	numSubsystems_ = numEventTimes_ + 1;

	// find active event times range: [activeEventTimeBeginIndex_, activeEventTimeEndIndex_)
	activeEventTimeBeginIndex_ = findActiveSubsystemIndex(eventTimes(), slqPtr_->initTime_);
	activeEventTimeEndIndex_   = findActiveSubsystemIndex(eventTimes(), slqPtr_->finalTime_);

	// display
	if (settingsPtr_->displayInfo_)
		std::cerr << "\n#### Calculating cost function sensitivity ..." << std::endl;

	// use the LQ-based method or Sweeping-BVP method
	if (settingsPtr_->useLQForDerivatives_==true)
		runLQBasedMethod();
	else
		runSweepingBVPMethod();
}


} // namespace ocs2

