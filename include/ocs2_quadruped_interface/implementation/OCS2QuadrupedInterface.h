/*
 * OCS2QuadrupedInterface.h
 *
 *  Created on: Feb 14, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedInterface<JOINT_COORD_SIZE>::OCS2QuadrupedInterface(
		const kinematic_model_t& kinematicModel,
		const com_model_t& comModel,
		const std::string& pathToConfigFolder)

: kinematicModelPtr_(kinematicModel.clone()),
  comModelPtr_(comModel.clone()),
  switchedModelStateEstimator_(comModel)

{
	// load sertting from loading file
	loadSettings(pathToConfigFolder);

	// logic rule
	logicRules_ = logic_rules_t(modelSettings_.swingLegLiftOff_, 1.0 /*swingTimeScale*/);
	logicRules_.setMotionConstraints(initSwitchingTimes_, initSwitchingModes_, gapIndicatorPtrs_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::loadSettings(const std::string& pathToConfigFile) {

	// load SLQ settings
	slqSettings_.loadSettings(pathToConfigFile, true);

	// load MPC settings
	mpcSettings_.loadSettings(pathToConfigFile, true);

	// load switched model settings
	modelSettings_.loadSettings(pathToConfigFile, true);

	std::cerr << std::endl;

	scalar_t dt, finalTime, initSettlingTime;
	loadSimulationSettings(pathToConfigFile, dt, finalTime, initSettlingTime);

	initTime_ = 0.0;
	finalTime_ = finalTime;

	// initial state of the switched system
	ocs2::loadEigenMatrix(pathToConfigFile, "initialRobotState", initRbdState_);
	computeSwitchedModelState(initRbdState_, initSwitchedState_);

	// cost function components
	ocs2::loadEigenMatrix(pathToConfigFile, "Q", Q_);
	ocs2::loadEigenMatrix(pathToConfigFile, "R", R_);
	ocs2::loadEigenMatrix(pathToConfigFile, "Q_final", QFinal_);
	// target state
	base_coordinate_t comFinalPose;
	ocs2::loadEigenMatrix(pathToConfigFile, "CoM_final_pose", comFinalPose);
	xFinal_ = initSwitchedState_;
	xFinal_.template head<6>() += comFinalPose;

	// load the switchingModes
	loadSwitchingModes(pathToConfigFile, initSwitchingModes_, true);
	initNumSubsystems_ = initSwitchingModes_.size();
	// display
	std::cerr << "Initial Switching Modes: {";
	for (const auto& switchingMode: initSwitchingModes_)
		std::cerr << switchingMode << ", ";
	std::cerr << "\b\b}" << std::endl;

	// stanceLeg sequence
	initStanceLegSequene_.resize(initNumSubsystems_);
	for (size_t i=0; i<initNumSubsystems_; i++)  initStanceLegSequene_[i] = modeNumber2StanceLeg(initSwitchingModes_[i]);

	// Initial switching times
	size_t NumNonFlyingSubSystems=0;
	for (size_t i=0; i<initNumSubsystems_; i++)
		if (initSwitchingModes_[i] != FLY)
			NumNonFlyingSubSystems++;
	initSwitchingTimes_.resize(initNumSubsystems_);
	initSwitchingTimes_.front() = initTime_;
	for (size_t i=0; i<initNumSubsystems_-1; i++)
		if (initSwitchingModes_[i] != FLY)
			initSwitchingTimes_[i+1] = initSwitchingTimes_[i] + (finalTime_-initTime_)/NumNonFlyingSubSystems;
		else
			initSwitchingTimes_[i+1] = initSwitchingTimes_[i] + 0.2;
	initSwitchingTimes_.erase(initSwitchingTimes_.begin());
	// display
	std::cerr << "Initial Switching Times: {";
	for (const auto& switchingtime: initSwitchingTimes_)
		std::cerr << switchingtime << ", ";
	if (initSwitchingTimes_.empty()==false)
		std::cerr << "\b\b}" << std::endl;
	else
		std::cerr << "}" << std::endl;

	// partitioning times
	partitioningTimes_.clear();
	partitioningTimes_.push_back(initTime_);
	for (const auto& t : initSwitchingTimes_)
		partitioningTimes_.push_back(t);
	partitioningTimes_.push_back(finalTime_);
	// display
	std::cerr << "Time Partition: {";
	for (const auto& timePartition: partitioningTimes_)
		std::cerr << timePartition << ", ";
	std::cerr << "\b\b}" << std::endl;
	// Number of partitioning times
	numPartitioningTimes_ = partitioningTimes_.size()-1;

	// Gap Indicators
	loadGaps(pathToConfigFile, gapIndicatorPtrs_, true);

	switchingModes_    = initSwitchingModes_;
	stanceLegSequene_  = initStanceLegSequene_;
	switchingTimes_    = initSwitchingTimes_;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getPerformanceIndeces(scalar_t& costFunction,
		scalar_t& constriantISE1,
		scalar_t& constriantISE2) const {

	costFunction = costFunction_;
	constriantISE1 = constriantISE1_;
	constriantISE2 = constriantISE2_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getController(controller_array_t& controllersStock) const {

	controllersStock = controllersStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<const typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::controller_array_t>
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getControllerPtr() const {

	return controllersStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<const std::vector<typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::scalar_array_t>>
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getTimeTrajectoriesPtr() const {

	return timeTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<const typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::state_vector_array2_t>
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getStateTrajectoriesPtr() const {

	return stateTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
std::shared_ptr<const typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::input_vector_array2_t>
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getInputTrajectoriesPtr() const {

	return inputTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getTrajectories(
		std::vector<scalar_array_t>& timeTrajectoriesStock,
		state_vector_array2_t& stateTrajectoriesStock,
		input_vector_array2_t& inputTrajectoriesStock) const {

	timeTrajectoriesStock  = timeTrajectoriesStock_;
	stateTrajectoriesStock = stateTrajectoriesStock_;
	inputTrajectoriesStock = inputTrajectoriesStock_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getSwitchingTimes(
		scalar_array_t& switchingTimes) const {

	switchingTimes = switchingTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getGaitSequence(
		size_array_t& gaitSequence) const {

	gaitSequence = switchingModes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getStanceLegSequene(
		std::vector<contact_flag_t>& stanceLegSequene) const {

	stanceLegSequene = stanceLegSequene_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getGapIndicatorPtrs(
		std::vector<EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs) const {

	gapIndicatorPtrs = gapIndicatorPtrs_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getIterationsLog(
		eigen_scalar_array_t& iterationCost,
		eigen_scalar_array_t& iterationISE1,
		eigen_scalar_array_t& ocs2Iterationcost) const {

	iterationCost = iterationCost_;
	iterationISE1 = iterationISE1_;
	ocs2Iterationcost = ocs2Iterationcost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
ocs2::MPC_Settings& OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getMpcSettings() {

	return mpcSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
ocs2::SLQ_Settings& OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getSlqSettings() {
	return slqPtr_->settings();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::concatenate()  {

	timeTrajectory_  = timeTrajectoriesStock_[0];
	stateTrajectory_ = stateTrajectoriesStock_[0];
	inputTrajectory_  = inputTrajectoriesStock_[0];
	for(size_t i=1; i<numSubsystems_; i++) {
		timeTrajectory_.insert(timeTrajectory_.end(), timeTrajectoriesStock_[i].begin(), timeTrajectoriesStock_[i].end());
		stateTrajectory_.insert(stateTrajectory_.end(), stateTrajectoriesStock_[i].begin(), stateTrajectoriesStock_[i].end());
		inputTrajectory_.insert(inputTrajectory_.end(), inputTrajectoriesStock_[i].begin(), inputTrajectoriesStock_[i].end());
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

	linInterpolateUff_.setTimeStamp(&controllerTimeTrajectory_);
	linInterpolateUff_.setData(&controllerFFTrajector_);

	linInterpolateK_.setTimeStamp(&controllerTimeTrajectory_);
	linInterpolateK_.setData(&controllerFBTrajectory_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::runSLQ(
		const scalar_t& initTime,
		const rbd_state_vector_t& initRbdState,
		const scalar_t& finalTime,
		const scalar_array_t& partitioningTimes /*=scalar_array_t()*/,
		const controller_array_t& initialControllersStock /*=controller_array_t()*/)  {

	initTime_ = initTime;
	finalTime_ = finalTime;
	computeSwitchedModelState(initRbdState, initSwitchedState_);

	if (partitioningTimes.empty()==false) {
		partitioningTimes_ = partitioningTimes;
		numPartitioningTimes_ = partitioningTimes.size()-1;
	}

	// reference trajectories
	designDesiredTrajectories(desiredTimeTrajectoriesStock_,
			desiredStateTrajectoriesStock_, desiredInputTrajectoriesStock_);

	// TODO: numSubsystems_ is not set
//	scalar_array_t costAnnealingStartTimes(numSubsystems_, switchingTimes_.front());
//	scalar_array_t costAnnealingFinalTimes(numSubsystems_, switchingTimes_.back());

	// run slqp
	if (initialControllersStock.empty()==true) {
		std::cerr << "Cold initialization." << std::endl;
		controllersStock_.clear();
		slqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_,
				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_, desiredInputTrajectoriesStock_);

	} else {
		std::cerr << "Warm initialization." << std::endl;
		controllersStock_ = initialControllersStock;
		slqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_, controllersStock_,
				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_, desiredInputTrajectoriesStock_);
	}
//	if (initialControllersStock.empty()==true) {
//		std::cerr << "Cold initialization." << std::endl;
//		lqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_, 0.0,
//				desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
//		lqPtr_->getController(controllersStock_);
//
//	} else {
//		std::cerr << "Warm initialization." << std::endl;
//		controllersStock_ = initialControllersStock;
//	}
//	slqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_,
//			controllersStock_, desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);

	// get the optimizer parametet
	getOptimizerParameters(slqPtr_);

	// get the optimizer outputs
	ocs2Iterationcost_.clear();
	slqPtr_->getIterationsLog(iterationCost_, iterationISE1_, iterationISE2_);
	slqPtr_->getPerformanceIndeces(costFunction_, constriantISE1_, constriantISE2_);
	slqPtr_->getController(controllersStock_);
	slqPtr_->getNominalTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);

//	concatenate();

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
template<class T>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getOptimizerParameters(
		const std::shared_ptr<T>& optimizerPtr) {

	optimizerPtr->getEventTimes(switchingTimes_);

	optimizerPtr->getSwitchedSystemIDs(switchingModes_);

	numSubsystems_ = switchingModes_.size();

	stanceLegSequene_.resize(numSubsystems_);
	for (size_t i=0; i<numSubsystems_; i++)
		stanceLegSequene_[i] = switched_model::modeNumber2StanceLeg(switchingModes_[i]);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
bool OCS2QuadrupedInterface<JOINT_COORD_SIZE>::runMPC(
		const scalar_t& initTime,
		const rbd_state_vector_t& initHyQState)  {

	initTime_ = initTime;
	computeSwitchedModelState(initHyQState, initSwitchedState_);

	// update controller
	bool controllerIsUpdated = mpcPtr_->run(initTime_, initSwitchedState_);

	// get the optimizer outputs
	controllersStockPtr_ = mpcPtr_->getControllerPtr();

	// get the optimizer outputs
	timeTrajectoriesStockPtr_  = mpcPtr_->getTimeTrajectoriesPtr();
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
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::computeSwitchedModelState(
		const rbd_state_vector_t& rbdState,
		state_vector_t& comkinoState) {

	switchedModelStateEstimator_.estimateComkinoModelState(rbdState, comkinoState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::setNewGoalStateMPC(
		const scalar_t& newGoalDuration,
		const state_vector_t& newGoalState) {

	mpcPtr_->setNewGoalState(newGoalDuration, newGoalState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::runOCS2(
		const scalar_t& initTime,
		const rbd_state_vector_t& initHyQState,
		const scalar_array_t& switchingTimes /*=scalar_array_t()*/)  {

//	if (switchingTimes.empty()==true)
//		switchingTimes_ = initSwitchingTimes_;
//	else
//		switchingTimes_ = switchingTimes;
//
//	initTime_ = initTime;
//	computeSwitchedModelState(initHyQState, initSwitchedState_);
//
//	// run ocs2
//	ocs2Ptr_->run(initTime_, initSwitchedState_, switchingTimes_.back(), initSystemStockIndexes_, switchingTimes_,
//			controller_array_t(),
//			desiredTimeTrajectoriesStock_, desiredStateTrajectoriesStock_);
//
//	// get the optimizer outputs
//	ocs2Ptr_->getOCS2IterationsLog(ocs2Iterationcost_);
//	ocs2Ptr_->getSLQIterationsLog(iterationCost_, iterationISE1_);
//	ocs2Ptr_->getCostFunction(costFunction_);
//	constriantISE_ = iterationISE1_.back()(0);
//	ocs2Ptr_->getSwitchingTimes(switchingTimes_);
//	ocs2Ptr_->getController(controllersStock_);
//	ocs2Ptr_->getNominalTrajectories(timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_);

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::loadSimulationSettings(
		const std::string& filename,
		scalar_t& dt,
		scalar_t& tFinal,
		scalar_t& initSettlingTime) {

	const scalar_t defaultInitSettlingTime = 1.5;
	boost::property_tree::ptree pt;

	try	{
		boost::property_tree::read_info(filename, pt);
		dt     = pt.get<scalar_t>("simulationSettings.dt");
		tFinal = pt.get<scalar_t>("simulationSettings.tFinal");
		initSettlingTime = pt.get<scalar_t>("simulationSettings.initSettlingTime", defaultInitSettlingTime);
	}
	catch (const std::exception& e){
		std::cerr << "Tried to open file " << filename << " but failed: " << std::endl;
		std::cerr<<"Error in loading simulation settings: " << e.what() << std::endl;
		throw;
	}
	std::cerr<<"Simulation Settings: " << std::endl;
	std::cerr<<"=====================================" << std::endl;
	std::cerr<<"Simulation time step ......... " << dt << std::endl;
	std::cerr<<"Controller simulation time ... [0, " << tFinal  << "]" << std::endl;
	std::cerr<<"initial settling time ......... " << initSettlingTime  << std::endl << std::endl;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::loadVisualizationSettings(
		const std::string& filename,
		scalar_t& slowdown,
		scalar_t& vizTime) {

	boost::property_tree::ptree pt;

	try	{
		boost::property_tree::read_info(filename, pt);
		slowdown = pt.get<scalar_t>("visualization.slowdown");
		vizTime = pt.get<scalar_t>("visualization.vizTime");
	}
	catch (const std::exception& e){
		std::cerr<<"Error in loading viz settings: " << e.what() << std::endl;
	}
}

} // end of namespace switched_model

