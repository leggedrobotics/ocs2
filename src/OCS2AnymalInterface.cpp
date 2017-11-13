/*
 * OCS2AnymalInterface.cpp
 *
 *  Created on: May 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 *
 * @param time
 * @param hyqState
 * @param comPose
 * @param comVelocities
 */
void OCS2AnymalInterface::fromHyqStateToComStateOrigin(const double& time,
		const Eigen::Matrix<double,36,1>& hyqState,
		com_coordinate_t& comPose,
		com_coordinate_t& comVelocities)  {

	// rotation matrix
	Eigen::Matrix3d b_R_o = hyq::SwitchedModelKinematics::RotationMatrixOrigintoBase(hyqState.head<3>());
	// switched hyq state
	dimension_t::state_vector_t switchedState;
	hyq::SwitchedModelStateEstimator::EstimateSwitchedModelState(time, hyqState, switchedState);
	comPose = switchedState.segment<6>(0);
	com_coordinate_t comLocalVelocities = switchedState.segment<6>(6);
	comVelocities.head<3>() = b_R_o.transpose() * comLocalVelocities.head<3>();
	comVelocities.tail<3>() = b_R_o.transpose() * comLocalVelocities.tail<3>();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::adjustFootZdirection(const double& time, std::array<Eigen::Vector3d,4>& origin_base2StanceFeet, std::array<bool,4>& stanceLegSequene) {

	size_t activeSubsystemIndexInStock = findActiveSubsystemIndex(time);
	stanceLegSequene = hyq::HyQMode::ModeNumber2StanceLeg(initSwitchingModes_[activeSubsystemIndexInStock]);

	for (size_t j=0; j<4; j++) {
		if (stanceLegSequene[j]==true)
			origin_base2StanceFeet[j] = origin_base2StanceFeetPrev_[j];
		else
			origin_base2StanceFeet[j](2) = plannedCPGs_[activeSubsystemIndexInStock][j]->calculatePosition(time);

		origin_base2StanceFeetPrev_[j] = origin_base2StanceFeet[j];
	}  // end of j loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getCostFuntion(double& costFunction, double& constriantISE) const {
	costFunction = costFunction_;
	constriantISE = constriantISE_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getSwitchingTimes(std::vector<double>& switchingTimes) const {
	switchingTimes = switchingTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getController(dimension_t::controller_array_t& controllersStock) const {
	controllersStock = controllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<const OCS2AnymalInterface::dimension_t::controller_array_t> OCS2AnymalInterface::getControllerPtr() const {
	return controllersStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<const std::vector<OCS2AnymalInterface::dimension_t::scalar_array_t>> OCS2AnymalInterface::getTimeTrajectoriesPtr() const {
	return timeTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<const OCS2AnymalInterface::dimension_t::state_vector_array2_t> OCS2AnymalInterface::getStateTrajectoriesPtr() const {
	return stateTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<const OCS2AnymalInterface::dimension_t::control_vector_array2_t> OCS2AnymalInterface::getInputTrajectoriesPtr() const {
	return inputTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getTrajectories(std::vector<dimension_t::scalar_array_t>& timeTrajectoriesStock,
		dimension_t::state_vector_array2_t& stateTrajectoriesStock,
		dimension_t::control_vector_array2_t& inputTrajectoriesStock) const {
	timeTrajectoriesStock  = timeTrajectoriesStock_;
	stateTrajectoriesStock = stateTrajectoriesStock_;
	inputTrajectoriesStock = inputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getStanceLegSequene(std::vector<std::array<bool,4>>& stanceLegSequene) const {
	stanceLegSequene = stanceLegSequene_;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getSwitchingModeSequence(std::vector<size_t>& switchingModes) const {
	switchingModes = switchingModes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getSystemStockIndexes(std::vector<size_t>& systemStockIndexes) const {
	systemStockIndexes = systemStockIndexes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getGapIndicatorPtrs(std::vector<hyq::EndEffectorConstraintBase::Ptr>& gapIndicatorPtrs) const {
	gapIndicatorPtrs = gapIndicatorPtrs_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::getMpcOptions(typename mpc_t::mpc_settings_t& mpcOptions){
	mpcOptions = mpcOptions_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::concatenate()  {

	// calculate state time derivative
	calculateStateDerivative(initSystemStockIndexes_, switchingTimes_, timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_,
			stateDerivativeTrajectoriesStock_);

	timeTrajectory_  = timeTrajectoriesStock_[0];
	stateTrajectory_ = stateTrajectoriesStock_[0];
	inputTrajectory_  = inputTrajectoriesStock_[0];
	stateDerivativeTrajectory_ = stateDerivativeTrajectoriesStock_[0];
	for(size_t i=1; i<numSubsystems_; i++) {
		timeTrajectory_.insert(timeTrajectory_.end(), timeTrajectoriesStock_[i].begin(), timeTrajectoriesStock_[i].end());
		stateTrajectory_.insert(stateTrajectory_.end(), stateTrajectoriesStock_[i].begin(), stateTrajectoriesStock_[i].end());
		inputTrajectory_.insert(inputTrajectory_.end(), inputTrajectoriesStock_[i].begin(), inputTrajectoriesStock_[i].end());
		stateDerivativeTrajectory_.insert(stateDerivativeTrajectory_.end(), stateDerivativeTrajectoriesStock_[i].begin(), stateDerivativeTrajectoriesStock_[i].end());
	}

	controllerTimeTrajectory_ = controllersStock_[0].time_;
	controllerFBTrajectory_   = controllersStock_[0].k_;
	controllerFFTrajector_    = controllersStock_[0].uff_;
	for(size_t i=1; i<numSubsystems_; i++) {
		controllerTimeTrajectory_.insert(controllerTimeTrajectory_.end(), controllersStock_[i].time_.begin(), controllersStock_[i].time_.end());
		controllerFBTrajectory_.insert(controllerFBTrajectory_.end(), controllersStock_[i].k_.begin(), controllersStock_[i].k_.end());
		controllerFFTrajector_.insert(controllerFFTrajector_.end(), controllersStock_[i].uff_.begin(), controllersStock_[i].uff_.end());
	}

	linInterpolateState_.setTimeStamp(&timeTrajectory_);
	linInterpolateState_.setData(&stateTrajectory_);

	linInterpolateInput_.setTimeStamp(&timeTrajectory_);
	linInterpolateInput_.setData(&inputTrajectory_);

	linInterpolateStateDerivative_.setTimeStamp(&timeTrajectory_);
	linInterpolateStateDerivative_.setData(&stateDerivativeTrajectory_);

	linInterpolateUff_.setTimeStamp(&controllerTimeTrajectory_);
	linInterpolateUff_.setData(&controllerFFTrajector_);

	linInterpolateK_.setTimeStamp(&controllerTimeTrajectory_);
	linInterpolateK_.setData(&controllerFBTrajectory_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::runSLQP(const double& initTime,
		const Eigen::Matrix<double,36,1>& initHyQState,
		const dimension_t::controller_array_t& initialControllersStock/*=dimension_t::controller_array_t()*/,
		const std::vector<double>& switchingTimes/*=std::vector<double>()*/)  {

	initTime_ = initTime;
	hyq::SwitchedModelStateEstimator::EstimateSwitchedModelState(initTime_, initHyQState, initSwitchedHyqState_);

	if (switchingTimes.empty()==true)
		switchingTimes_ = initSwitchingTimes_;
	else
		switchingTimes_ = switchingTimes;

	dimension_t::scalar_array_t costAnnealingStartTimes(numSubsystems_, switchingTimes_.front());
	dimension_t::scalar_array_t costAnnealingFinalTimes(numSubsystems_, switchingTimes_.back());

	if (initialControllersStock.empty()==true) {
		std::cerr << "Cold initialization." << std::endl;
		// run lqp
		lqpPtr_->run(initSystemStockIndexes_, switchingTimes_, 0.0,
				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
		// get controller
		lqpPtr_->getController(controllersStock_);

	} else {
		std::cerr << "Warm initialization." << std::endl;
//		controllersStock_ = initialControllersStock;
	}

	// run slqp
	slqpPtr_->run(initTime_, initSwitchedHyqState_, switchingTimes_.back(), initSystemStockIndexes_, switchingTimes_,
			controllersStock_, desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);

	// get the optimizer parametet
	getOptimizerParameters(slqpPtr_);

	// get the optimizer outputs
	slqpPtr_->getIterationsLog(iterationCost_, iterationISE1_);
	ocs2Iterationcost_.clear();
	slqpPtr_->getCostFuntion(costFunction_, constriantISE_);
	slqpPtr_->getSwitchingTimes(switchingTimes_);
	slqpPtr_->getController(controllersStock_);
	slqpPtr_->getNominalTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);

//	concatenate();

//	hyq::SwitchedModelKinematics::FeetPositionsBaseFrame(initSwitchedHyqState_.tail<12>(), origin_base2StanceFeetPrev_);
//	Eigen::Matrix3d o_R_b = hyq::SwitchedModelKinematics::RotationMatrixOrigintoBase(initSwitchedHyqState_.head<3>()).transpose();
//	for (size_t j=0; j<4; j++) origin_base2StanceFeetPrev_[j] = (o_R_b * origin_base2StanceFeetPrev_[j] + initHyQState.segment<3>(3)).eval();
//
//	feetZDirectionPlannerPtr_->planAllModes(switchingTimes_, plannedCPGs_);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template<class T>
void OCS2AnymalInterface::getOptimizerParameters(const std::shared_ptr<T>& optimizerPtr) {

	optimizerPtr->getSwitchingTimes(switchingTimes_);
	optimizerPtr->getSubsystemIndexes(systemStockIndexes_);
	numSubsystems_ = systemStockIndexes_.size();

	switchingModes_.resize(numSubsystems_);
	stanceLegSequene_.resize(numSubsystems_);
	for (size_t i=0; i<numSubsystems_; i++) {
		switchingModes_[i] = initSwitchingModes_[systemStockIndexes_[i]];
		stanceLegSequene_[i] = initStanceLegSequene_[systemStockIndexes_[i]];
	}
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool OCS2AnymalInterface::runMPC(const double& initTime, const Eigen::Matrix<double,36,1>& initHyQState)  {

	initTime_ = initTime;
	hyq::SwitchedModelStateEstimator::EstimateSwitchedModelState(initTime_, initHyQState, initSwitchedHyqState_);

	// update controller
	bool controllerIsUpdated = mpcPtr_->run(initTime_, initSwitchedHyqState_);

	// get the optimizer outputs
	controllersStockPtr_ = mpcPtr_->getControllerPtr();

	// get the optimizer outputs
	timeTrajectoriesStockPtr_ = mpcPtr_->getTimeTrajectoriesPtr();
	stateTrajectoriesStockPtr_ = mpcPtr_->getStateTrajectoriesPtr();
	inputTrajectoriesStockPtr_ = mpcPtr_->getInputTrajectoriesPtr();

	// get the optimizer parametet: switchingTimes_, systemStockIndexes_,
	// numSubsystems_, switchingModes_, and stanceLegSequene_
	getOptimizerParameters(mpcPtr_);

	return controllerIsUpdated;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool OCS2AnymalInterface::runMPC(const double& initTime, const dimension_t::state_vector_t& initHyQState)  {

	initTime_ = initTime;
	initSwitchedHyqState_ = initHyQState;

	// update controller
	bool controllerIsUpdated = mpcPtr_->run(initTime_, initSwitchedHyqState_);

	// get the optimizer outputs
	mpcPtr_->getController(controllersStock_);

	// get the optimizer outputs
	mpcPtr_->getTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);

	// get the optimizer parametet: switchingTimes_, systemStockIndexes_,
	// numSubsystems_, switchingModes_, and stanceLegSequene_
	getOptimizerParameters(mpcPtr_);

	return controllerIsUpdated;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::setNewGoalStateMPC(const dimension_t::scalar_t& newGoalDuration, const dimension_t::state_vector_t& newGoalState) {
	mpcPtr_->setNewGoalState(newGoalDuration, newGoalState);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::runOCS2(const double& initTime,
		const Eigen::Matrix<double,36,1>& initHyQState,
		const std::vector<double>& switchingTimes/*=std::vector<double>()*/)  {

	if (switchingTimes.empty()==true)
		switchingTimes_ = initSwitchingTimes_;
	else
		switchingTimes_ = switchingTimes;

	initTime_ = initTime;
	hyq::SwitchedModelStateEstimator::EstimateSwitchedModelState(initTime_, initHyQState, initSwitchedHyqState_);

	// run ocs2
	ocs2Ptr_->run(initTime_, initSwitchedHyqState_, switchingTimes_.back(), initSystemStockIndexes_, switchingTimes_,
			dimension_t::controller_array_t(),
			desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);

	// get the optimizer outputs
	ocs2Ptr_->getOCS2IterationsLog(ocs2Iterationcost_);
	ocs2Ptr_->getSLQIterationsLog(iterationCost_, iterationISE1_);
	ocs2Ptr_->getCostFunction(costFunction_);
	constriantISE_ = iterationISE1_.back()(0);
	ocs2Ptr_->getSwitchingTimes(switchingTimes_);
	ocs2Ptr_->getController(controllersStock_);
	ocs2Ptr_->getNominalTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);

//	concatenate();
}



/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::loadSettings(const std::string& pathToConfigFolder, const Eigen::Matrix<double,36,1>& initHyQState) {
	std::string taskFile;
	taskFile = pathToConfigFolder + "/task.info";

	double dt, finalTime, initSettlingTime;
	loadSimulationSettings(taskFile, dt, finalTime, initSettlingTime);

	// gravity vector
	loadMatrix(taskFile,"gravity",gravity_);
	std::cerr <<"====================" << std::endl;
	std::cerr << "Gravity: \t" << gravity_.transpose().format(CleanFmtDisplay_) << std::endl << std::endl;

	// Initial state of the switchedModel
	hyq::SwitchedModelStateEstimator::EstimateSwitchedModelState(0.0, initHyQState, initSwitchedHyqState_);

	// cost function components
	loadMatrix(taskFile, "Q", Q_);
	loadMatrix(taskFile, "R", R_);
	loadMatrix(taskFile, "Q_final", QFinal_);
	// target state
	dimension_t::state_vector_t xFinalLoaded;
	loadMatrix(taskFile, "x_final", xFinalLoaded);
	xFinal_ = initSwitchedHyqState_;
	xFinal_.head<6>() += xFinalLoaded.head<6>();

	// OCS2 options
	ocs2::loadOptions<12+12,12+12>(taskFile, slqpOptions_, true);

	// MPC settings
	ocs2::loadMpcSettings<12+12,12+12>(taskFile, mpcOptions_, true);

	// load switched model options
	loadModelSettings(taskFile, options_, zmpWeight_, impulseWeight_, impulseSigmeFactor_, true);

	// load the switchingModes
	hyq::loadSwitchingModes(taskFile, initSwitchingModes_);
	initNumSubsystems_ = initSwitchingModes_.size();
	std::cerr << "initSwitchingModes: "
			<< Eigen::Matrix<size_t,1,-1>::Map(initSwitchingModes_.data(),initNumSubsystems_).format(CleanFmtDisplay_) << std::endl;

	// stanceLeg sequence
	initStanceLegSequene_.resize(initNumSubsystems_);
	for (size_t i=0; i<initNumSubsystems_; i++)  initStanceLegSequene_[i] = hyq::HyQMode::ModeNumber2StanceLeg(initSwitchingModes_[i]);

	// subsystems index in the subsystem-stock
	initSystemStockIndexes_.resize(initNumSubsystems_);
	for (size_t i=0; i<initNumSubsystems_; i++)  initSystemStockIndexes_[i] = i;

	// Initial switching times
	size_t NumNonFlyingSubSystems=0;
	for (size_t i=0; i<initNumSubsystems_; i++)
		if (initSwitchingModes_[initSystemStockIndexes_[i]] != hyq::FLY)
			NumNonFlyingSubSystems++;
	initSwitchingTimes_.resize(initNumSubsystems_+1);
	initSwitchingTimes_.front() = 0.0;
	initSwitchingTimes_.back()  = finalTime-initSettlingTime;
	for (size_t i=0; i<initNumSubsystems_-1; i++)
		if (initSwitchingModes_[initSystemStockIndexes_[i]] != hyq::FLY)
			initSwitchingTimes_[i+1] = initSwitchingTimes_[i] + (finalTime-initSettlingTime)/NumNonFlyingSubSystems;
		else
			initSwitchingTimes_[i+1] = initSwitchingTimes_[i] + 0.2;
	std::cerr << "initSwitchingTimes: " << Eigen::RowVectorXd::Map(initSwitchingTimes_.data(),initNumSubsystems_+1).format(CleanFmtDisplay_)
					<< std::endl << std::endl;

	// Gap Indicators
	hyq::loadGaps(taskFile, gapIndicatorPtrs_, true);

	// mrt_deadzone
	loadMatrix(taskFile, "mrt_deadzone", mrtDeadzone_);

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::setupOptimizaer()  {

	// Z direction CPG planner
	feetZDirectionPlannerPtr_ = hyq::FeetZDirectionPlanner<z_direction_cpg_t>::Ptr(new hyq::FeetZDirectionPlanner<z_direction_cpg_t>(
			initStanceLegSequene_, options_.swingLegLiftOff_, (initSwitchingTimes_[1]-initSwitchingTimes_[0]) /*time scale for adjusting z hight*/,
			anymal::Mode::ModeNumber2StanceLeg(options_.defaultStartMode_),
			anymal::Mode::ModeNumber2StanceLeg(options_.defaultFinalMode_) ) );

	// for each subsystem that defined in stanceLegSequene
	subsystemDynamicsPtr_.resize(initNumSubsystems_);
	subsystemDerivativesPtr_.resize(initNumSubsystems_);
	subsystemCostFunctionsPtr_.resize(initNumSubsystems_);
	stateOperatingPoints_.resize(initNumSubsystems_);
	inputOperatingPoints_.resize(initNumSubsystems_);
	controllersStock_.resize(initNumSubsystems_);
	desiredTimeTrajectoriesStock_.resize(initNumSubsystems_);
	desiredStateTrajectoriesStock_.resize(initNumSubsystems_);
	for (size_t i=0; i<initNumSubsystems_; i++) {

		// nominal time
		dimension_t::scalar_array_t tNominalTrajectory(2);
		tNominalTrajectory[0] = initSwitchingTimes_.front();
		tNominalTrajectory[1] = initSwitchingTimes_.back();
		desiredTimeTrajectoriesStock_[i] = tNominalTrajectory;
		// nominal state
		dimension_t::state_vector_array_t xNominalTrajectory(2);
		xNominalTrajectory[0] = initSwitchedHyqState_;
		xNominalTrajectory[1] = xFinal_;
		desiredStateTrajectoriesStock_[i] = xNominalTrajectory;
		// nominal control inputs for weight compensation
		dimension_t::control_vector_t uNominalForWeightCompensation;
		std::array<Eigen::Vector3d,4> sphericalWeightCompensationForces;
		Eigen::Matrix3d b_R_o = hyq::SwitchedModelKinematics::RotationMatrixOrigintoBase(initSwitchedHyqState_.head<3>());
		if (options_.useCartesianContactForce_==false)
			WeightCompensationForces::ComputeSphericalForces(b_R_o*gravity_, std::array<bool,4>{1,1,1,1}/*initStanceLegSequene_[i]*/, initSwitchedHyqState_.tail<12>(),
					sphericalWeightCompensationForces);
		else
			WeightCompensationForces::ComputeCartesianForces(b_R_o*gravity_, std::array<bool,4>{1,1,1,1}/*initStanceLegSequene_[i]*/, initSwitchedHyqState_.tail<12>(),
					sphericalWeightCompensationForces);
		for (size_t j=0; j<4; j++)
			uNominalForWeightCompensation.segment<3>(3*j) = sphericalWeightCompensationForces[j];
		uNominalForWeightCompensation.tail<12>().setZero();
		dimension_t::control_vector_array_t uNominalTrajectory(2, uNominalForWeightCompensation);

		// state and input operating points
		stateOperatingPoints_[i] = initSwitchedHyqState_;
		inputOperatingPoints_[i] = uNominalForWeightCompensation;
		if (options_.constrainedIntegration_==false)  {
			inputOperatingPoints_[i].setZero();
			WeightCompensationForces::ComputeSphericalForces(b_R_o*gravity_, std::array<bool,4>{1,1,1,1}, initSwitchedHyqState_.tail<12>(), sphericalWeightCompensationForces);
			for (size_t j=0; j<4; j++)  inputOperatingPoints_[i].segment<3>(3*j) = sphericalWeightCompensationForces[j];
		}

		// R matrix
		dimension_t::control_matrix_t nondiagonalR = R_;
		double meanRz = (R_(2,2)+R_(5,5)+R_(8,8)+R_(11,11))/4;
		for (int j=0; j<4; j++)
			for (int k=0; k<=j; k++)
				if (k==j) {
					nondiagonalR(3*j+0,3*k+0) = 1.05*R_(3*j+0,3*k+0);
					nondiagonalR(3*j+1,3*k+1) = 1.05*R_(3*j+1,3*k+1);
					nondiagonalR(3*j+2,3*k+2) = 1.05*meanRz;
				}
				else {
					if (initStanceLegSequene_[i][j] && initStanceLegSequene_[i][k])
						nondiagonalR(3*j+2,3*k+2) = 2*meanRz;
					else
						nondiagonalR(3*j+2,3*k+2) = 0.0;
				}
		nondiagonalR = 0.5*(nondiagonalR + nondiagonalR.transpose()).eval();

		subsystemCostFunctionsPtr_[i] = std::shared_ptr<cost_funtion_t>( new cost_funtion_t(initStanceLegSequene_[i], Q_, nondiagonalR,
				desiredTimeTrajectoriesStock_[i], desiredStateTrajectoriesStock_[i], uNominalTrajectory, QFinal_, xFinal_, zmpWeight_, impulseWeight_, impulseSigmeFactor_) );

		// subsystem settings
		subsystemDynamicsPtr_[i]    = std::shared_ptr<system_dynamics_t>( new system_dynamics_t(initStanceLegSequene_[i], -gravity_(2), options_,
				feetZDirectionPlannerPtr_, gapIndicatorPtrs_) );
		subsystemDerivativesPtr_[i] = std::shared_ptr<system_dynamics_derivative_t>( new system_dynamics_derivative_t(initStanceLegSequene_[i], -gravity_(2), options_,
				feetZDirectionPlannerPtr_, gapIndicatorPtrs_) );

	}  // end of i loop

	switchingModes_     = initSwitchingModes_;
	stanceLegSequene_   = initStanceLegSequene_;
	systemStockIndexes_ = initSystemStockIndexes_;
	switchingTimes_     = initSwitchingTimes_;

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void OCS2AnymalInterface::calculateStateDerivative(const std::vector<size_t>& systemStockIndexes,
		const std::vector<double>& switchingTimes,
		const std::vector<dimension_t::scalar_array_t>& timeTrajectoriesStock,
		const dimension_t::state_vector_array2_t& stateTrajectoriesStock,
		const dimension_t::control_vector_array2_t& inputTrajectoriesStock,
		dimension_t::state_vector_array2_t& stateDerivativeTrajectoriesStock)  {


	stateDerivativeTrajectoriesStock.resize(numSubsystems_);
	for (int i=0; i<numSubsystems_; i++) {

		std::shared_ptr<system_dynamics_t::Base> currSubsystemDynamicsPtr = subsystemDynamicsPtr_[initSystemStockIndexes_[i]]->clone();

		// initialize subsystem i
		dimension_t::state_vector_t x0 = stateTrajectoriesStock[i][0];
		currSubsystemDynamicsPtr->initializeModel(systemStockIndexes, switchingTimes, x0, i, "GSLPQ");

		// compute state derivative trajectory for subsystem i
		stateDerivativeTrajectoriesStock[i].resize(timeTrajectoriesStock[i].size());
		for (int k=0; k<timeTrajectoriesStock[i].size(); k++)  {
			currSubsystemDynamicsPtr->computeDerivative(timeTrajectoriesStock[i][k], stateTrajectoriesStock[i][k], inputTrajectoriesStock[i][k],
					stateDerivativeTrajectoriesStock[i][k]);
		}
	}
}
