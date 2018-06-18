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
	feet_z_planner_ptr_t feetZPlannerPtr( new feet_z_planner_t(modelSettings_.swingLegLiftOff_, 1.0 /*swingTimeScale*/) );

	logicRulesPtr_ = logic_rules_ptr_t( new logic_rules_t(feetZPlannerPtr) );

	logicRulesPtr_->setMotionConstraints(initSwitchingModes_, initEventTimes_, gapIndicatorPtrs_);
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
	std::cerr << std::endl;
	loadModes(pathToConfigFile, "switchingModes", initSwitchingModes_, true);
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
	initEventTimes_.resize(initNumSubsystems_);
	initEventTimes_.front() = initTime_;
	for (size_t i=0; i<initNumSubsystems_-1; i++)
		if (initSwitchingModes_[i] != FLY)
			initEventTimes_[i+1] = initEventTimes_[i] + (finalTime_-initTime_)/NumNonFlyingSubSystems;
		else
			initEventTimes_[i+1] = initEventTimes_[i] + 0.2;
	initEventTimes_.erase(initEventTimes_.begin());
	// display
	std::cerr << "Initial Event Times:     {";
	for (const auto& switchingtime: initEventTimes_)
		std::cerr << switchingtime << ", ";
	if (initEventTimes_.empty()==false)
		std::cerr << "\b\b}" << std::endl;
	else
		std::cerr << "}" << std::endl;

	// partitioning times
	partitioningTimes_.clear();
	partitioningTimes_.push_back(initTime_);
	for (const auto& t : initEventTimes_)
		partitioningTimes_.push_back(t);
	partitioningTimes_.push_back(finalTime_);
	// display
	std::cerr << "Time Partition: {";
	for (const auto& timePartition: partitioningTimes_)
		std::cerr << timePartition << ", ";
	std::cerr << "\b\b}" << std::endl;
	// Number of partitioning times
	numPartitioningTimes_ = partitioningTimes_.size()-1;

	// load the mode sequence template
	std::cerr << std::endl;
	loadModes(pathToConfigFile, "templateSubsystemsSequence", modeSequenceTemplate_.templateSubsystemsSequence_, true);
	loadStdVector(pathToConfigFile, "templateSwitchingTimes", modeSequenceTemplate_.templateSwitchingTimes_, true);
	std::cerr << std::endl;

	// Gap Indicators
	loadGaps(pathToConfigFile, gapIndicatorPtrs_, true);

	switchingModes_    = initSwitchingModes_;
	stanceLegSequene_  = initStanceLegSequene_;
	switchingTimes_    = initEventTimes_;

	// Ground profile class
	scalar_t groundHight;
	estimateFlatGround(initRbdState_, contact_flag_t{1,1,1,1}, groundHight);
	std::cerr << "Ground Profile Hight: " << groundHight << std::endl << std::endl;
	groundProfilePtr_ = flat_ground_profile_ptr_t( new flat_ground_profile_t(groundHight) );
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::logic_rules_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getLogicRules() {

	return *logicRulesPtr_;
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
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::computeRbdModelState(
		const state_vector_t& comkinoState,
		const input_vector_t& comkinoInput,
		rbd_state_vector_t& rbdState) {

	switchedModelStateEstimator_.estimateRbdModelState(comkinoState, comkinoInput.template tail<JOINT_COORD_SIZE>(),
			rbdState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::computeComLocalAcceleration(
			const state_vector_t& comkinoState,
			const input_vector_t& comkinoInput,
			base_coordinate_t& comLocalAcceleration) {

	typedef Eigen::Matrix<scalar_t,3,3> matrix_3_t;
	typedef Eigen::Matrix<scalar_t,3,1> vector_3_t;
	typedef Eigen::Matrix<scalar_t,6,6> matrix_6_t;
	typedef Eigen::Matrix<scalar_t,6,1> vector_6_t;

	static vector_3_t o_gravityVector = vector_3_t(0.0, 0.0, -modelSettings_.gravitationalAcceleration_);

	Eigen::VectorBlock<const state_vector_t,6> xCOM  = comkinoState.template segment<6>(0);
	Eigen::VectorBlock<const state_vector_t,3> b_W_com = comkinoState.template segment<3>(6);
	Eigen::VectorBlock<const state_vector_t,3> b_V_com = comkinoState.template segment<3>(9);
	Eigen::VectorBlock<const state_vector_t,12> qJoints  = comkinoState.template tail<12>();
	Eigen::VectorBlock<const input_vector_t,12> dqJoints = comkinoInput.template tail<12>();
	Eigen::VectorBlock<const input_vector_t,3*4> lambda  = comkinoInput.template head<12>();

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(xCOM.template head<3>());

	// base to CoM displacement in the CoM frame
	vector_3_t b_base2CoM = comModelPtr_->comPositionBaseFrame(qJoints);

	// base coordinate
	base_coordinate_t xBase;
	xBase.template head<3>() = xCOM.template head<3>();
	xBase.template tail<3>() = xCOM.template tail<3>() - o_R_b * b_base2CoM;

	// update kinematic model
	kinematicModelPtr_->update(xBase, qJoints);

	// base to stance feet displacement in the CoM frame
	std::array<vector_3_t,4> b_base2StanceFeet;
	for (size_t i=0; i<4; i++)
		kinematicModelPtr_->footPositionBaseFrame(i, b_base2StanceFeet[i]);

	// Inertia matrix in the CoM frame and its derivatives
	matrix_6_t M    = comModelPtr_->comInertia(qJoints);
	matrix_6_t dMdt = comModelPtr_->comInertiaDerivative(qJoints, dqJoints);
	matrix_3_t rotationMInverse = M.template topLeftCorner<3,3>().inverse();
	matrix_6_t MInverse;
	MInverse << rotationMInverse, 	matrix_3_t::Zero(),
				matrix_3_t::Zero(), (1/M(5,5))*matrix_3_t::Identity();

	// Coriolis and centrifugal forces
	vector_6_t C;
	C.template head<3>() = b_W_com.cross(M.template topLeftCorner<3,3>()*b_W_com) + dMdt.template topLeftCorner<3,3>()*b_W_com;
	C.template tail<3>().setZero();

	// gravity effect on CoM in CoM coordinate
	vector_6_t MInverseG;
	MInverseG << vector_3_t::Zero(), -o_R_b.transpose() * o_gravityVector;

	// contact JacobianTransposeLambda
	vector_3_t b_comToFoot;
	vector_6_t JcTransposeLambda = vector_6_t::Zero();
	for (size_t i=0; i<4; i++)  {

		b_comToFoot = b_base2StanceFeet[i] - b_base2CoM;

		JcTransposeLambda.template head<3>() += b_comToFoot.cross(lambda.template segment<3>(3*i));
		JcTransposeLambda.template tail<3>() += lambda.template segment<3>(3*i);
	}

	// CoM acceleration about CoM frame
	comLocalAcceleration = MInverse * (-C + JcTransposeLambda) - MInverseG;

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::computeComStateInOrigin(
		const state_vector_t& comkinoState,
		const input_vector_t& comkinoInput,
		base_coordinate_t& o_comPose,
		base_coordinate_t& o_comVelocity,
		base_coordinate_t& o_comAcceleration)  {

	Eigen::VectorBlock<const state_vector_t,3> o_r_com = comkinoState.template segment<3>(3);
	Eigen::VectorBlock<const state_vector_t,3> b_W_com = comkinoState.template segment<3>(6);
	Eigen::VectorBlock<const state_vector_t,3> b_V_com = comkinoState.template segment<3>(9);

	// compute CoM local acceleration about CoM frame
	base_coordinate_t comLocalAcceleration;
	computeComLocalAcceleration(comkinoState, comkinoInput, comLocalAcceleration);

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = kinematicModelPtr_->rotationMatrixOrigintoBase().transpose();

	// CoM pose in the origin frame
	o_comPose << comkinoState.template head<3>(), o_r_com;

	// CoM velocity in the origin frame
	o_comVelocity.template head<3>() = o_R_b *  b_W_com;
	o_comVelocity.template tail<3>() = o_R_b *  b_V_com;
//	o_comVelocity.template tail<3>() = o_R_b * (b_V_com - b_W_com.cross(o_r_com));

	// CoM acceleration in the origin frame
	o_comAcceleration.template head<3>() = o_R_b *  comLocalAcceleration.template head<3>();
	o_comAcceleration.template tail<3>() = o_R_b *  comLocalAcceleration.template tail<3>();
//	o_comAcceleration.template tail<3>() = o_R_b * (comLocalAcceleration.template tail<3>() - comLocalAcceleration.template head<3>().cross(o_r_com)
//			- b_W_com.cross(b_W_com.cross(o_r_com)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::estimateFlatGround(
		const rbd_state_vector_t& rbdState,
		const contact_flag_t& contactFlag,
		scalar_t& groundHight) const {

	kinematicModelPtr_->update(rbdState.template head<6+JOINT_COORD_SIZE>());

	std::array<Eigen::Vector3d,4> feetPositions;
	kinematicModelPtr_->feetPositionsOriginFrame(feetPositions);

	groundHight = 0.0;
	for (size_t j=0; j<4; j++)
		if (contactFlag[j]==true)
			groundHight += 0.25*feetPositions[j](2);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::kinematic_model_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getKinematicModel() {

	return *kinematicModelPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::com_model_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getComModel() {

	return *comModelPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::slq_base_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getSLQ() {

	return *slqPtr_;
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::slq_base_ptr_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getSLQPtr() {

	return slqPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::mpc_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getMPC() {

	return *mpcPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
typename OCS2QuadrupedInterface<JOINT_COORD_SIZE>::mpc_ptr_t&
	OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getMPCPtr() {

	return mpcPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getPerformanceIndeces(
		scalar_t& costFunction,
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
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getOptimizedControllerPtr(
		const controller_array_t*& controllersStockPtr) const {

	controllersStockPtr = controllersStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getOptimizedTrajectoriesPtr(
		const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr) const {

	timeTrajectoriesStockPtr = timeTrajectoriesStockPtr_;
	stateTrajectoriesStockPtr = stateTrajectoriesStockPtr_;
	inputTrajectoriesStockPtr = inputTrajectoriesStockPtr_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getEventTimesPtr(
		const scalar_array_t*& eventTimesPtr) const {

	eventTimesPtr = &eventTimes_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getSubsystemsSequencePtr(
		const size_array_t*& subsystemsSequencePtr) const {

	subsystemsSequencePtr = &subsystemsSequence_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getContactFlagsSequencePtr(
		const std::vector<contact_flag_t>*& contactFlagsSequencePtr) const {

	contactFlagsSequencePtr = &contactFlagsSequence_;
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
Model_Settings& OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getModelSettings() {

	return modelSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::getLoadedInitialState(rbd_state_vector_t& initRbdState) const {

	initRbdState = initRbdState_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedInterface<JOINT_COORD_SIZE>::concatenate()  {

	timeTrajectory_  = timeTrajectoriesStockPtr_->at(0);
	stateTrajectory_ = stateTrajectoriesStockPtr_->at(0);
	inputTrajectory_  = inputTrajectoriesStockPtr_->at(0);
	for(size_t i=1; i<numSubsystems_; i++) {
		timeTrajectory_.insert(timeTrajectory_.end(), timeTrajectoriesStockPtr_->at(i).begin(), timeTrajectoriesStockPtr_->at(i).end());
		stateTrajectory_.insert(stateTrajectory_.end(), stateTrajectoriesStockPtr_->at(i).begin(), stateTrajectoriesStockPtr_->at(i).end());
		inputTrajectory_.insert(inputTrajectory_.end(), inputTrajectoriesStockPtr_->at(i).begin(), inputTrajectoriesStockPtr_->at(i).end());
	}

	controllerTimeTrajectory_ = controllersStockPtr_->at(0).time_;
	controllerFBTrajectory_   = controllersStockPtr_->at(0).k_;
	controllerFFTrajector_    = controllersStockPtr_->at(0).uff_;
	for(size_t i=1; i<numSubsystems_; i++) {
		controllerTimeTrajectory_.insert(controllerTimeTrajectory_.end(), controllersStockPtr_->at(i).time_.begin(), controllersStockPtr_->at(i).time_.end());
		controllerFBTrajectory_.insert(controllerFBTrajectory_.end(), controllersStockPtr_->at(i).k_.begin(), controllersStockPtr_->at(i).k_.end());
		controllerFFTrajector_.insert(controllerFFTrajector_.end(), controllersStockPtr_->at(i).uff_.begin(), controllersStockPtr_->at(i).uff_.end());
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
	input_vector_t uNominalForWeightCompensation;
	designWeightCompensatingInput(initSwitchedState_, uNominalForWeightCompensation);

	// reference time
	costDesiredTrajectories_.desiredTimeTrajectory().resize(2);
	costDesiredTrajectories_.desiredTimeTrajectory().at(0) = initEventTimes_.front();
	costDesiredTrajectories_.desiredTimeTrajectory().at(1) = initEventTimes_.back();
	// reference state
	costDesiredTrajectories_.desiredStateTrajectory().resize(2);
	costDesiredTrajectories_.desiredStateTrajectory().at(0) = initSwitchedState_;
	costDesiredTrajectories_.desiredStateTrajectory().at(1) = xFinal_;
	// reference inputs for weight compensation
	costDesiredTrajectories_.desiredInputTrajectory().resize(2);
	costDesiredTrajectories_.desiredInputTrajectory().at(0) = uNominalForWeightCompensation;
	costDesiredTrajectories_.desiredInputTrajectory().at(1) = uNominalForWeightCompensation;

	slqPtr_->setCostDesiredTrajectories(costDesiredTrajectories_);

	// TODO: numSubsystems_ is not set
//	scalar_array_t costAnnealingStartTimes(numSubsystems_, switchingTimes_.front());
//	scalar_array_t costAnnealingFinalTimes(numSubsystems_, switchingTimes_.back());

	// run slqp
	if (initialControllersStock.empty()==true) {
		std::cerr << "Cold initialization." << std::endl;
		slqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_);

	} else {
		std::cerr << "Warm initialization." << std::endl;
		slqPtr_->run(initTime_, initSwitchedState_, finalTime_, partitioningTimes_, initialControllersStock);
	}

	// get the optimizer outputs
	ocs2Iterationcost_.clear();
	slqPtr_->getIterationsLog(iterationCost_, iterationISE1_, iterationISE2_);
	slqPtr_->getPerformanceIndeces(costFunction_, constriantISE1_, constriantISE2_);

	slqPtr_->getControllerPtr(controllersStockPtr_);
	slqPtr_->getNominalTrajectoriesPtr(
			timeTrajectoriesStockPtr_,
			stateTrajectoriesStockPtr_,
			inputTrajectoriesStockPtr_);

	// get gait sequence (should be copied since it might be overridden in the next iteration)
	eventTimes_ = slqPtr_->getLogicRules().eventTimes();
	subsystemsSequence_ = slqPtr_->getLogicRules().subsystemsSequence();
	contactFlagsSequence_ = slqPtr_->getLogicRules().getContactFlagsSequence();

//	concatenate();

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
bool OCS2QuadrupedInterface<JOINT_COORD_SIZE>::runMPC(
		const scalar_t& initTime,
		const rbd_state_vector_t& initState)  {

	initTime_ = initTime;
	computeSwitchedModelState(initState, initSwitchedState_);

	// update controller
	bool controllerIsUpdated = mpcPtr_->run(initTime_, initSwitchedState_);

	// get the optimizer outputs
	mpcPtr_->getOptimizedControllerPtr(controllersStockPtr_);

	mpcPtr_->getOptimizedTrajectoriesPtr(
			timeTrajectoriesStockPtr_,
			stateTrajectoriesStockPtr_,
			inputTrajectoriesStockPtr_);

	// get gait sequence (should be copied since it might be overridden in the next iteration)
	eventTimes_ = mpcPtr_->getLogicRules().eventTimes();

	subsystemsSequence_ = mpcPtr_->getLogicRules().subsystemsSequence();
//	contactFlagsSequence_ = mpcPtr_->getLogicRules().getContactFlagsSequence();

	return controllerIsUpdated;
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
//		switchingTimes_ = initEventTimes_;
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

