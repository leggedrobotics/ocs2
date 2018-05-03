/*
 * OCS2QuadrupedMRT.h
 *
 *  Created on: Mar 7, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedMRT<JOINT_COORD_SIZE>::OCS2QuadrupedMRT(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const std::string& robotName /*robot*/)

: ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr),
  robotName_(robotName),
  modelSettings_(ocs2QuadrupedInterfacePtr->getModelSettings()),
  mpcSettings_(ocs2QuadrupedInterfacePtr->getMpcSettings())
{
	reset();

	//get mpc settings
	if (mpcSettings_.recedingHorizon_==false)
		mpcSettings_.rosMsgTimeWindow_ = 1e+6;

	// logic machine
	feetZDirectionPlannerPtr_  	= feet_z_planner_ptr_t( new feet_z_planner_t(modelSettings_.swingLegLiftOff_, 1.0 /*swingTimeScale*/) );
	logicRulesPtr_   			= logic_rules_ptr_t( new logic_rules_t(feetZDirectionPlannerPtr_) );
	logicMachinePtr_ 			= logic_machine_ptr_t( new logic_machine_t(*logicRulesPtr_) );

	// Start thread for publishing
#ifdef PUBLISH_THREAD
	updateNodesThread_ = std::thread(&OCS2QuadrupedMRT::updateNodesThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedMRT<JOINT_COORD_SIZE>::~OCS2QuadrupedMRT() {

#ifdef PUBLISH_THREAD
	std::cerr << "Shutting down workers ..." << std::endl;

	std::unique_lock<std::mutex> lk(updateNodesMutex_);
	terminateThread_ = true;
	lk.unlock();

	updateReady_.notify_all();

	if (updateNodesThread_.joinable())
		updateNodesThread_.join();

	std::cerr << "All workers shut down." << std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::reset() {

	timeStep_ = 0.0;

	planReceivedEver_ = false;
	planRecived_ 	= false;
	replanMRT_ 		= false;
	timeIsFreezed_ 	= false;

	partitioningTimes_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
bool OCS2QuadrupedMRT<JOINT_COORD_SIZE>::initialPlanReceived() const {

	return planReceivedEver_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::sigintHandler(int sig)  {

	ROS_INFO_STREAM("Shutting MRT node.");
	::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::launchNodes(int argc, char* argv[]) {

	reset();

	// display
	ROS_INFO_STREAM("MRT node is setting up ...");

	// setup ROS
	::ros::init(argc, argv, robotName_+"_mrt", ::ros::init_options::NoSigintHandler);
	signal(SIGINT, OCS2QuadrupedMRT::sigintHandler);
	::ros::NodeHandle nodeHandler;

	// state pulisher
	statePublisher_ = nodeHandler.advertise<ocs2_ros_msg::rbd_state_vector>(robotName_+"_state", 1);

	// dummy publisher
	dummyPublisher_ = nodeHandler.advertise<ocs2_ros_msg::dummy>("ping_mrt", 1, true);

	// SLQ controller subscriber
	if (mpcSettings_.useFeedbackPolicy_==true)
		slqControllerSubscriber_   = nodeHandler.subscribe(robotName_+"_slq_ctlr", 1, &OCS2QuadrupedMRT::slqpControllerCallback, this);
	else
		slqTrajectoriesSubscriber_ = nodeHandler.subscribe(robotName_+"_slq_trjc", 1, &OCS2QuadrupedMRT::optimizedTrajectoryCallback, this);

	// wait for one second
	::ros::spinOnce();
	sleep(1);

	ROS_INFO_STREAM("MRT node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::publishDummy() {

	ocs2_ros_msg::dummy message;
	message.ping = 1;
	dummyPublisher_.publish(message);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::publishCurrentState(
		const contact_flag_t& contactFlag,
		const scalar_t& time,
		const rbd_state_vector_t& rbdState) const {

//	if (planReceivedEver_==true)  return;  // for debuging

	if (rbdState != rbdState) {
		std::cerr << "WARNING: corrupted data in HyQ state." << std::endl;
		return;
	}

	if (contactFlag != contactFlag) {
		std::cerr << "WARNING: corrupted data in stance leg flags." << std::endl;
		return;
	}

	static size_t numCalls = 0;
	numCalls++;

	// creat the message
	ocs2_ros_msg::rbd_state_vector msg;
	msg.timeStamp = time;
	msg.header.seq = numCalls;
	for (size_t i=0; i<RBD_STATE_DIM; i++)
		msg.state[i] = rbdState(i);
	for (size_t i=0; i<4; i++)
		msg.contactFlag[i] = contactFlag[i];

	statePublisher_.publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::optimizedTrajectoryCallback (
		const ocs2_ros_msg::switched_model_trajectory::ConstPtr& msg) {

	planInitTime_  = msg->planInitTime;
	planInitState_ = (Eigen::Map<const Eigen::Matrix<float,RBD_STATE_DIM,1>>(msg->planInitState.data())).template cast<scalar_t>();
	bool controllerIsUpdated = msg->controllerIsUpdated;

	std::cout << "\t Plan is recieved at time: " << planInitTime_ << std::endl;

	bool isLogicUpdated = extractMotionSequence(msg->gaitSequence,
			eventTimes_, phaseSequence_);

	const size_t NE = phaseSequence_.size();
	touchdownTimeStock_.clear();
	touchdownTimeStock_.reserve(NE+1);
	touchdownStateStock_.clear();
	touchdownStateStock_.reserve(NE+1);
	touchdownInputStock_.clear();
	touchdownInputStock_.reserve(NE+1);

	const size_t N = msg->timeTrajectory.size();
	timeTrajectory_.clear();
	timeTrajectory_.reserve(N);
	stateTrajectory_.clear();
	stateTrajectory_.reserve(N);
	inputTrajectory_.clear();
	inputTrajectory_.reserve(N);

	for (size_t i=0; i<N; i++) {

		timeTrajectory_.push_back(
				msg->timeTrajectory[i]);
		stateTrajectory_.push_back(
				Eigen::Map<const Eigen::Matrix<float,STATE_DIM,1>>(msg->stateTrajectory[i].state.data()).template cast<scalar_t>() );
		inputTrajectory_.push_back(
				Eigen::Map<const Eigen::Matrix<float,INPUT_DIM,1>>(msg->inputTrajectory[i].input.data()).template cast<scalar_t>() );

		// save touchdown information
		if (i==0 || (timeTrajectory_.back()-msg->timeTrajectory[i-1])<ocs2::OCS2NumericTraits<scalar_t>::week_epsilon()) {
			touchdownTimeStock_.push_back(timeTrajectory_.back());
			touchdownStateStock_.push_back(stateTrajectory_.back());
			touchdownInputStock_.push_back(inputTrajectory_.back());
			// making the reference and the measured EE velocity the same
			if (i==0) touchdownInputStock_[0].template tail<12>() = planInitState_.template tail<12>();
		}
	} // end of i loop
	touchdownTimeStock_.push_back(timeTrajectory_.back());
	touchdownStateStock_.push_back(stateTrajectory_.back());
	touchdownInputStock_.push_back(inputTrajectory_.back());

	// time partitioning
	scalar_array_t partitioningTimesTemp{timeTrajectory_.front(), timeTrajectory_.back()};
	if (partitioningTimesTemp != partitioningTimes_) {
		isLogicUpdated = true;
		partitioningTimes_ = std::move( partitioningTimesTemp );
	}

	std::unique_lock<std::mutex> lk(optimizedTrajectoryMutex_);

	if (isLogicUpdated==true) {
		updateLogics(eventTimes_, phaseSequence_, partitioningTimes_,
				touchdownTimeStock_, touchdownStateStock_, touchdownInputStock_);
	}

	linInterpolateState_.setTimeStamp(&timeTrajectory_);
	linInterpolateState_.setData(&stateTrajectory_);

	linInterpolateInput_.setTimeStamp(&timeTrajectory_);
	linInterpolateInput_.setData(&inputTrajectory_);

	lk.unlock();

	if (controllerIsUpdated==true) {
		planReceivedEver_ = true;
		planRecived_ = true;
	}

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::slqpControllerCallback (
		const ocs2_ros_msg::slq_controller_trajectory::ConstPtr& msg) {

	planInitTime_  = msg->planInitTime;
	planInitState_ = (Eigen::Map<const Eigen::Matrix<float,RBD_STATE_DIM,1>>(msg->planInitState.data())).template cast<scalar_t>();
	bool controllerIsUpdated = msg->controllerIsUpdated;

	bool isLogicUpdated = extractMotionSequence(msg->gaitSequence,
			eventTimes_, phaseSequence_);

	const size_t N = msg->slqControllerTrajectory.size();

	input_vector_t uffTemp;
	control_feedback_t kTemp;
	slqController_.clear();
	slqController_.time_.reserve(N);
	slqController_.uff_.reserve(N);
	slqController_.k_.reserve(N);

	for (size_t i=0; i<N; i++){

		uffTemp = (Eigen::Map<const Eigen::Matrix<float,INPUT_DIM,1>>(msg->slqControllerTrajectory[i].uff.data())).template cast<scalar_t>();

		//covert from array to Eigen types
		for(size_t p=0; p<INPUT_DIM; p++)
			for(size_t q=0; q<STATE_DIM; q++)
				kTemp(p,q) = msg->slqControllerTrajectory[i].gainMatrix[p * INPUT_DIM + q];

		slqController_.time_.push_back(msg->slqControllerTrajectory[i].timeStamp);
		slqController_.uff_.push_back(uffTemp);
		slqController_.k_.push_back(kTemp);
	} //end i loop

	if (controllerIsUpdated==true) {
		planReceivedEver_ = true;
		planRecived_ = true;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
template <class ContainerAllocator>
bool OCS2QuadrupedMRT<JOINT_COORD_SIZE>::extractMotionSequence(
		const ocs2_ros_msg::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
		scalar_array_t& eventTimes,
		size_array_t& phaseSequence) const {

	const size_t numSubsystems = gaitSequenceMsg.phaseSequence.size();
	if (gaitSequenceMsg.eventTimes.size() != numSubsystems-1)
		throw std::runtime_error("The received message has incompatible array sizes for the eventTimes and phaseSequence.");

	scalar_array_t eventTimesTemp(numSubsystems-1);
	size_array_t phaseSequenceTemp(numSubsystems);

	for (size_t i=0; i<numSubsystems-1; i++) {
		eventTimesTemp[i]    = gaitSequenceMsg.eventTimes[i];
		phaseSequenceTemp[i] = gaitSequenceMsg.phaseSequence[i];
	}
	phaseSequenceTemp.back() = gaitSequenceMsg.phaseSequence.back();

	bool isLogicUpdated = false;
	if (eventTimesTemp != eventTimes){
		eventTimes = std::move(eventTimesTemp);
		isLogicUpdated = true;
	}
	if (phaseSequenceTemp != phaseSequence){
		phaseSequence = std::move(phaseSequenceTemp);
		isLogicUpdated = true;
	}

	return isLogicUpdated;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::updateLogics(
		const scalar_array_t& eventTimes,
		const size_array_t& phaseSequence,
		const scalar_array_t& partitioningTimes,
		const scalar_array_t& touchdownTimeStock,
		const state_vector_array_t& touchdownStateStock,
		const input_vector_array_t& touchdownInputStock) {

	// compute feet
	const size_t n = touchdownTimeStock.size();
	vector_3d_array_t o_contactForcesTemp;
	std::vector<vector_3d_array_t> o_feetPositionStock(n);
	std::vector<vector_3d_array_t> o_feetVelocityStock(n);
	for (size_t i=0; i<n; i++) {

		computeFeetState(touchdownStateStock[i], touchdownInputStock[i],
				o_feetPositionStock[i], o_feetVelocityStock[i], o_contactForcesTemp);

		if (0<i && i<n-1 )
			o_feetVelocityStock[i].fill(vector_3d_t::Zero());

	}  // end of i loop

	{
		// display
		std::cout << "\t touchdownTimeStock: {";
		for (const auto& t: touchdownTimeStock)
			std::cout << t << ", ";
		std::cout << "\b\b}" << std::endl;
		std::cout << "\t touchdownTimeStock.size: " << touchdownTimeStock.size() << std::endl;

		std::cout << "\t eventTimes: {";
		for (const auto& t: eventTimes)
			std::cout << t << ", ";
		std::cout << "\b\b}" << std::endl;
		std::cout << "\t eventTimes.size: " << eventTimes.size() << std::endl;

		std::cout << "\t phaseSequence: {";
		for (const auto& t: phaseSequence)
			std::cout << t << ", ";
		std::cout << "\b\b}" << std::endl;
		std::cout << "\t phaseSequence.size: " << phaseSequence.size() << std::endl;
	}

	if (partitioningTimes.size()>2)
		throw std::runtime_error("There should be only one time partitioning.");

	// set gait settings
	logicRulesPtr_->setModeSequence(phaseSequence, eventTimes);

	// set updated logicRules
	logicMachinePtr_->setLogicRules(*logicRulesPtr_);

	// update logicMachine
	controller_array_t controllerStockTemp(0);
	logicMachinePtr_->updateLogicRules(partitioningTimes, controllerStockTemp);

	{
		// display
		logicMachinePtr_->display();
	}

	// function for finding active subsystem
	const size_t partitionIndex = 0; // we assume only one partition
	findActiveSubsystemFnc_ = std::move( logicMachinePtr_->getHandleToFindActiveEventCounter(partitionIndex) );

	const size_t numPhaseSequence = phaseSequence.size();
	size_t initActiveSubsystem  = findActiveSubsystemFnc_(touchdownTimeStock.front());
	size_t finalActiveSubsystem = findActiveSubsystemFnc_(touchdownTimeStock.back());

	// XY plan
	feetXPlanPtrStock_.resize(numPhaseSequence);
	feetYPlanPtrStock_.resize(numPhaseSequence);
	for (size_t i=0; i<numPhaseSequence; i++) {

		if (i<initActiveSubsystem || finalActiveSubsystem<i ) {
			continue;
		}

		const size_t startIndex = i - initActiveSubsystem;

		for (size_t j=0; j<4; j++) {

			feetXPlanPtrStock_[i][j] = std::make_shared<cubic_spline_t>();
			feetYPlanPtrStock_[i][j] = std::make_shared<cubic_spline_t>();

			feetXPlanPtrStock_[i][j]->set(
					touchdownTimeStock[startIndex] /*t0*/, o_feetPositionStock[startIndex][j](0) /*p0*/, o_feetVelocityStock[startIndex][j](0) /*v0*/,
					touchdownTimeStock[startIndex+1] /*t1*/, o_feetPositionStock[startIndex+1][j](0) /*p1*/, o_feetVelocityStock[startIndex+1][j](0) /*v1*/);

			feetYPlanPtrStock_[i][j]->set(
					touchdownTimeStock[startIndex] /*t0*/, o_feetPositionStock[startIndex][j](1) /*p0*/, o_feetVelocityStock[startIndex][j](1) /*v0*/,
					touchdownTimeStock[startIndex+1] /*t1*/, o_feetPositionStock[startIndex+1][j](1) /*p1*/, o_feetVelocityStock[startIndex+1][j](1) /*v1*/);
		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::computeFeetState(
		const state_vector_t& state,
		const input_vector_t& input,
		vector_3d_array_t& o_feetPosition,
		vector_3d_array_t& o_feetVelocity,
		vector_3d_array_t& o_contactForces)  {

	base_coordinate_t comPose = state.template head<6>();
	base_coordinate_t comLocalVelocities = state.template segment<6>(6);
	joint_coordinate_t qJoints  = state.template tail<12>();
	joint_coordinate_t dqJoints = input.template tail<12>();

	base_coordinate_t basePose;
	ocs2QuadrupedInterfacePtr_->getComModel().calculateBasePose(qJoints, comPose, basePose);
	base_coordinate_t baseLocalVelocities;
	ocs2QuadrupedInterfacePtr_->getComModel().calculateBaseLocalVelocities(qJoints, dqJoints, comLocalVelocities, baseLocalVelocities);

	ocs2QuadrupedInterfacePtr_->getKinematicModel().update(basePose, qJoints);
	Eigen::Matrix3d o_R_b = ocs2QuadrupedInterfacePtr_->getKinematicModel().rotationMatrixOrigintoBase().transpose();

	for(size_t i=0; i<4; i++) {
		// calculates foot position in the base frame
		vector_3d_t b_footPosition;
		ocs2QuadrupedInterfacePtr_->getKinematicModel().footPositionBaseFrame(i, b_footPosition);

		// calculates foot position in the origin frame
		o_feetPosition[i] = o_R_b * b_footPosition + basePose.template tail<3>();

		// calculates foot velocity in the base frame
		Eigen::Matrix<scalar_t,6,JOINT_COORD_SIZE> b_footJacobain;
		ocs2QuadrupedInterfacePtr_->getKinematicModel().footJacobainBaseFrame(i, b_footJacobain);
		vector_3d_t b_footVelocity = (b_footJacobain*dqJoints).template tail<3>();

		// calculates foot velocity in the origin frame
		ocs2QuadrupedInterfacePtr_->getKinematicModel().FromBaseVelocityToInertiaVelocity(
				o_R_b, baseLocalVelocities, b_footPosition, b_footVelocity, o_feetVelocity[i]);

		// calculates contact forces in the origin frame
		o_contactForces[i] = o_R_b * input.template segment<3>(3*i);

	} // end of i loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::updateNodes(
		const contact_flag_t& contactFlag,
		const scalar_t& time,
		const rbd_state_vector_t& rbdState,
		const scalar_t& mcLoopFrequency /*= 250*/) {

	timeStep_ = 1.0 / mcLoopFrequency;

#ifdef PUBLISH_THREAD

	std::unique_lock<std::mutex> lk(updateNodesMutex_);

	currentContactFlag_ = contactFlag;
	currentTime_     = time;
	currentRbdState_ = rbdState;

	readyToUpdate_ = true;
	lk.unlock();
	updateReady_.notify_one();

#else

	updateNodesWorker(contactFlag, time, rbdState);

#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::updateNodesThread() {

	while(true) {

		std::unique_lock<std::mutex> lk(updateNodesMutex_);

		updateReady_.wait(lk, [&]{return (readyToUpdate_ || terminateThread_);});

		if (terminateThread_==true)  break;

		currentContactFlagBuffer_ = std::move( currentContactFlag_ );
		currentTimeBuffer_ = std::move( currentTime_ );
		currentRbdStateBuffer_ = std::move( currentRbdState_ );

		readyToUpdate_ = false;

		lk.unlock();

		// Send data
		updateNodesWorker(currentContactFlagBuffer_,
				currentTimeBuffer_,
				currentRbdStateBuffer_);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::updateNodesWorker(
		const contact_flag_t& contactFlag,
		const scalar_t& time,
		const rbd_state_vector_t& rbdState) {

	static size_t mpcloopCounter = 0;

	// for testing asynchronous MPC loop
	size_t numIterToWaitForMPC = std::floor(mpcSettings_.simulatedMPCloopTime_ / (timeStep_ * 1000));

	if (numIterToWaitForMPC == 0  ||  mpcloopCounter % numIterToWaitForMPC == 0) {

		// publish the current time and state
		publishCurrentState(contactFlag, time, rbdState);

		// check for a new controller update from the MPC node
		::ros::spinOnce();

		// for testing asynchronous MPC loop
//		if (numIterToWaitForMPC != 0  && mpcloopCounter % numIterToWaitForMPC == 0) {
//			if (mpcloopCounter==0)  std::cout << std::endl << "WARNING: The MPC loop is running asynchronously." << std::endl;
//			// check for a new controller update from the MPC node
//			while (planRecived_==false && correctedTime_ <= switchingTimes_.tail(1)(0))
//				::ros::spinOnce();
//		}
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::computePlan(
		const scalar_t& time,
		vector_3d_array_t& o_feetPositionRef,
		vector_3d_array_t& o_feetVelocityRef,
		vector_3d_array_t& o_feetAccelerationRef,
		base_coordinate_t& o_comPoseRef,
		base_coordinate_t& o_comVelocityRef,
		base_coordinate_t& o_comAccelerationRef,
		contact_flag_t& stanceLegs)  {

	std::cout << "computePlan called at " << time << std::endl;

	std::unique_lock<std::mutex> lk(optimizedTrajectoryMutex_);

	// optimal switched model state and input
	linInterpolateState_.interpolate(time, stateRef_);
	int greatestLessTimeStampIndex = linInterpolateState_.getGreatestLessTimeStampIndex();
	linInterpolateInput_.interpolate(time, inputRef_, greatestLessTimeStampIndex);

//	// calculates nominal position, velocity, and contact forces of the feet in the origin frame.
//	// This also updates the kinematic model.
//	computeFeetState(stateRef, stateRef, o_feetPositionRef, o_feetVelocityRef, o_contactForcesRef);

	// filter swing leg trajectory
	size_t index = findActiveSubsystemFnc_(time);
	logicRulesPtr_->getMotionPhaseLogics(index, stanceLegs, feetZPlanPtr_);

	for (size_t j=0; j<4; j++) {
		o_feetPositionRef[j] <<
				feetXPlanPtrStock_[index][j]->evaluateSplinePosition(time),
				feetYPlanPtrStock_[index][j]->evaluateSplinePosition(time),
				feetZPlanPtr_[j]->calculatePosition(time);

		o_feetVelocityRef[j] <<
				feetXPlanPtrStock_[index][j]->evaluateSplineVelocity(time),
				feetYPlanPtrStock_[index][j]->evaluateSplineVelocity(time),
				feetZPlanPtr_[j]->calculateVelocity(time);

		o_feetAccelerationRef[j] <<
				feetXPlanPtrStock_[index][j]->evaluateSplineAcceleration(time),
				feetYPlanPtrStock_[index][j]->evaluateSplineAcceleration(time),
				feetZPlanPtr_[j]->calculateAcceleration(time);
	}

	lk.unlock();

	// calculate CoM pose, velocity, and acceleration in the origin frame.
	ocs2QuadrupedInterfacePtr_->computeComStateInOrigin(stateRef_, inputRef_,
			o_comPoseRef, o_comVelocityRef, o_comAccelerationRef);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMRT<JOINT_COORD_SIZE>::getComputedReferences(
		state_vector_t& stateRef,
		input_vector_t& inputRef) const {

	stateRef = stateRef_;
	inputRef = inputRef_;
}


} // end of namespace switched_model

