/*
 * OCS2QuadrupedMPC.h
 *
 *  Created on: Mar 1, 2018
 *      Author: farbod
 */

namespace switched_model {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedMPC<JOINT_COORD_SIZE>::OCS2QuadrupedMPC(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const std::string& robotName /*robot*/)

		: ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr),
		  robotName_(robotName)
{
	reset();

	//get mpc settings
	mpcSettings_ = ocs2QuadrupedInterfacePtr_->getMpcSettings();
	if (mpcSettings_.recedingHorizon_==false)
		mpcSettings_.rosMsgTimeWindow_ = 1e+6;

	// Start thread for publishing
#ifdef PUBLISH_THREAD
	publisherWorker_ = std::thread(&OCS2QuadrupedMPC::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
OCS2QuadrupedMPC<JOINT_COORD_SIZE>::~OCS2QuadrupedMPC() {

#ifdef PUBLISH_THREAD
	std::cerr << "Shutting down workers ..." << std::endl;

	std::unique_lock<std::mutex> lk(publishMutex_);
	stopPublishing_ = true;
	lk.unlock();

	msgReady_.notify_all();

	if (publisherWorker_.joinable())
		publisherWorker_.join();

	std::cerr << "All workers are shut down." << std::endl;
#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::sigintHandler(int sig)  {

	ROS_INFO_STREAM("Shutting MPC node.");
	::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::reset() {

	initialCall_ = true;
	numIterations_ = 0;

	maxDelay_ = -1e6;
	meanDelay_ = 0.0;
	currentDelay_ = 0.0;

	targetPositionIsUpdated_ = true;
	targetReachingDuration_ = 1.0;
	targetPoseDisplacement_ = base_coordinate_t::Zero();

	stopPublishing_ = false;
	readyToPublish_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::publishDummy() {

	ocs2_ros_msg::dummy message;
	message.ping = 1;
	dummyPublisher_.publish(message);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::publishOptimizeTrajectories(
		const scalar_t& planInitTime,
		const rbd_state_vector_t& planInitState,
		const bool& controllerIsUpdated,
		const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr,
		const scalar_array_t*& eventTimesPtr,
		const size_array_t*& phaseSequencePtr)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publishMutex_);
#endif

	switchedModelTrajectoryMsg_.controllerIsUpdated  = controllerIsUpdated;
	switchedModelTrajectoryMsg_.planInitTime = planInitTime;
	for (size_t i=0; i<RBD_STATE_DIM; i++)
		switchedModelTrajectoryMsg_.planInitState[i] = planInitState(i);

	gaitSequenceMsg(eventTimesPtr, phaseSequencePtr,
			switchedModelTrajectoryMsg_.gaitSequence);

	switchedModelTrajectoryMsg_.timeTrajectory.clear();
	switchedModelTrajectoryMsg_.stateTrajectory.clear();
	switchedModelTrajectoryMsg_.inputTrajectory.clear();

	const scalar_t t0 = planInitTime + currentDelay_*1e-3;
	const scalar_t tf = planInitTime + mpcSettings_.rosMsgTimeWindow_*1e-3;
	if (tf < t0+2.0*meanDelay_*1e-3)
		std::cout << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;

	int lastActiveSubsystem = -1;
	for (size_t i=0; i<timeTrajectoriesStockPtr->size(); i++)  {

		const scalar_array_t& timeTrajectory        = timeTrajectoriesStockPtr->at(i);
		const state_vector_array_t& stateTrajectory = stateTrajectoriesStockPtr->at(i);
		const input_vector_array_t& inputTrajectory = inputTrajectoriesStockPtr->at(i);

		size_t N = timeTrajectory.size();
		if (N == 0)  continue;
		if (timeTrajectory.back()  < t0)  continue;
		if (timeTrajectory.front() > tf)  continue;

		lastActiveSubsystem = i;
		for (size_t k=0; k<N; k++) {
			// continue if elapsed time is smaller than computation time delay
			if (k<N-1 && timeTrajectory[k+1] < t0)  continue;
			// break if the time exceed rosMsgTimeWindow
			if (timeTrajectory[k] > tf)  break;

			ocs2_ros_msg::switched_state stateBoost;
			for (size_t j=0; j<STATE_DIM; j++)
				stateBoost.state[j] = stateTrajectory[k](j);
			ocs2_ros_msg::switched_input inputBoost;
			for (size_t j=0; j<INPUT_DIM; j++)
				inputBoost.input[j] = inputTrajectory[k](j);

			switchedModelTrajectoryMsg_.timeTrajectory.push_back(timeTrajectory[k]);
			switchedModelTrajectoryMsg_.stateTrajectory.push_back(stateBoost);
			switchedModelTrajectoryMsg_.inputTrajectory.push_back(inputBoost);

		}  // end of k loop
	}  // end of i loop

#ifdef PUBLISH_THREAD
	readyToPublish_ = true;
	lk.unlock();
	msgReady_.notify_one();
#else
	slqTrajectoriesPublisher_.publish(switchedModelTrajectoryMsg_);
#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::publishSlqController(
		const scalar_t& planInitTime,
		const rbd_state_vector_t& planInitState,
		const bool& controllerIsUpdated,
		const controller_array_t*& controllerStockPtr,
		const scalar_array_t*& eventTimesPtr,
		const size_array_t*& phaseSequencePtr)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publishMutex_);
#endif
	ocs2_ros_msg::slq_controller slqControllerMsg;
	bool nanFound = false;

	slqControllerTrajectoryMsg_.dimInput = INPUT_DIM;
	slqControllerTrajectoryMsg_.dimState = STATE_DIM;

	slqControllerTrajectoryMsg_.controllerIsUpdated = controllerIsUpdated;
	slqControllerTrajectoryMsg_.planInitTime = planInitTime;
	for (size_t i=0; i<RBD_STATE_DIM; i++)
		slqControllerTrajectoryMsg_.planInitState[i] = planInitState(i);

	gaitSequenceMsg(eventTimesPtr, phaseSequencePtr,
			slqControllerTrajectoryMsg_.gaitSequence);

	//
	slqControllerTrajectoryMsg_.slqControllerTrajectory.clear();

	const scalar_t t0 = planInitTime + currentDelay_*1e-3;
	const scalar_t tf = planInitTime + mpcSettings_.rosMsgTimeWindow_*1e-3;
	if (tf < t0+2.0*meanDelay_*1e-3)
		std::cout << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;

	int lastActiveSubsystem = -1;
	for (size_t i=0; i<controllerStockPtr->size(); i++)  {

		const controller_t& controller = controllerStockPtr->at(i);

		size_t N = controller.time_.size();

		if (N == 0)  continue;
		if (controller.time_.back()  < t0)  continue;
		if (controller.time_.front() > tf)  continue;

		lastActiveSubsystem = i;
		for (size_t k=0; k<N; k++) {

			// continue if elapsed time is smaller than computation time delay
			if (k<N-1 && controller.time_[k+1] < t0)  continue;
			// break if the time exceed rosMsgTimeWindow
			if (controller.time_[k] > tf)  break;

			slqControllerMsg.timeStamp = controller.time_[k];
//			if ( std::isnan(controllerTrajectoriesStock->at(i).time_[k]) )  nanFound = true;

			for (size_t p=0; p<INPUT_DIM; p++) {
				slqControllerMsg.uff[p] = controller.uff_[k](p);
				for (size_t q=0; q<STATE_DIM; q++)
					slqControllerMsg.gainMatrix[p*INPUT_DIM + q] = controller.k_[k](p,q);
			} // end of p loop

			slqControllerTrajectoryMsg_.slqControllerTrajectory.push_back(slqControllerMsg);

		}  // end of k loop

	}  // end of i loop

#ifdef PUBLISH_THREAD
	readyToPublish_ = true;
	lk.unlock();
	msgReady_.notify_one();
#else
	slqControllerPublisher_.publish(slqControllerTrajectoryMsg_);
#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
template <class ContainerAllocator>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::gaitSequenceMsg(
		const scalar_array_t*& eventTimesPtr,
		const size_array_t*& phaseSequencePtr,
		ocs2_ros_msg::gait_sequence_<ContainerAllocator>& gaitSequenceMsg) const {

	//
	gaitSequenceMsg.eventTimes.clear();
	gaitSequenceMsg.eventTimes.reserve(eventTimesPtr->size());
	for (const scalar_t& ti: *eventTimesPtr)
		gaitSequenceMsg.eventTimes.push_back(ti);

	//
	gaitSequenceMsg.phaseSequence.clear();
	gaitSequenceMsg.phaseSequence.reserve(phaseSequencePtr->size());
	for (const size_t& pi: *phaseSequencePtr)
		gaitSequenceMsg.phaseSequence.push_back(pi);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::stateCallback(const ocs2_ros_msg::rbd_state_vector::ConstPtr& msg) {

	// current time and state
	scalar_t currentTime = msg->timeStamp;
	rbd_state_vector_t rbdCurrentState = Eigen::Map<const Eigen::Matrix<float,RBD_STATE_DIM,1>> (msg->state.data()).template cast<scalar_t>();

	if (mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_)
		startTimePoint_ = std::chrono::steady_clock::now();

	// number of iterations
	numIterations_++;

	// calibrate the reference for target tracking
	if (initialCall_==true) {

		initialCall_ = false;

		// the default state
		ocs2QuadrupedInterfacePtr_->computeSwitchedModelState(rbdCurrentState, initState_);
		// Designs weight compensating input.
		ocs2QuadrupedInterfacePtr_->designWeightCompensatingInput(initState_, initInput_);

		// set the goal
		if (mpcSettings_.recedingHorizon_==true) {
			targetPositionIsUpdated_ = true;
			targetPoseDisplacement_.setZero();
			targetReachingDuration_ = 1.0;
		} else {
			targetPositionIsUpdated_ = false;
		}
	}

	// update target position
	if(targetPositionIsUpdated_==true) {

//		targetPositionIsUpdated_ = false;
//
//		state_vector_t currentState;
//		ocs2QuadrupedInterfacePtr_->computeSwitchedModelState(rbdCurrentState, currentState);
//
//		state_vector_t targetState;
//		targetState.template segment<3>(0) = initState_. template segment<3>(0)   + targetPoseDisplacement_.template segment<3>(0);
//		targetState.template segment<2>(3) = currentState. template segment<2>(3) + targetPoseDisplacement_.template segment<2>(3);
//		targetState.template segment<1>(5) = initState_. template segment<1>(5)   + targetPoseDisplacement_.template segment<1>(5);
//		targetState.template segment<6>(6).setZero();
//		targetState.template segment<12>(12) = initState_.template segment<12>(12);
//
//		ocs2QuadrupedInterfacePtr_->setNewGoalStateMPC(targetReachingDuration_, targetState);

		targetPositionIsUpdated_ = false;

		state_vector_t currentState;
		ocs2QuadrupedInterfacePtr_->computeSwitchedModelState(rbdCurrentState, currentState);

		// nominal time
		tGoalTrajectory_.resize(2);
		tGoalTrajectory_[0] = currentTime + ocs2QuadrupedInterfacePtr_->getModelSettings().mpcGoalCommandDelay_;
		tGoalTrajectory_[1] = currentTime + ocs2QuadrupedInterfacePtr_->getModelSettings().mpcGoalCommandDelay_ + targetReachingDuration_;

		// nominal state
		xGoalTrajectory_.resize(2);
		xGoalTrajectory_[0].template head<6>()     = currentState.template head<6>();
		xGoalTrajectory_[0].template segment<6>(6) = base_coordinate_t::Zero();
		xGoalTrajectory_[0].template tail<12>()    = initState_.template tail<12>();

		xGoalTrajectory_[1].template segment<3>(0) = initState_. template segment<3>(0)   + targetPoseDisplacement_.template segment<3>(0);
		xGoalTrajectory_[1].template segment<2>(3) = currentState. template segment<2>(3) + targetPoseDisplacement_.template segment<2>(3);
		xGoalTrajectory_[1].template segment<1>(5) = initState_. template segment<1>(5)   + targetPoseDisplacement_.template segment<1>(5);
		xGoalTrajectory_[1].template segment<6>(6) = base_coordinate_t::Zero();
		xGoalTrajectory_[1].template tail<12>()    = initState_.template tail<12>();

		// nominal input
		uGoalTrajectory_.resize(2);
		uGoalTrajectory_[0] = initInput_;
		uGoalTrajectory_[1] = initInput_;

		ocs2QuadrupedInterfacePtr_->getMPC().setNewGoalTrajectories(tGoalTrajectory_, xGoalTrajectory_, uGoalTrajectory_);

		// display
		std::cerr << "### The target position is updated at time " << std::setprecision(4) << currentTime << " to: \t [";
		for (size_t i=0; i<3; i++)
			std::cerr << xGoalTrajectory_[1].template segment<3>(3)(i) << ", ";
		std::cerr << "\b\b]" << std::endl;

	} else if (mpcSettings_.recedingHorizon_==false) {
		return;
	}

	// run SLQ-MPC
	bool controllerIsUpdated = ocs2QuadrupedInterfacePtr_->runMPC(currentTime, rbdCurrentState);
	// get optimized controller
	const controller_array_t* controllersStockPtr(nullptr);
	ocs2QuadrupedInterfacePtr_->getOptimizedControllerPtr(controllersStockPtr);
	// get optimized trajectories
	const std::vector<scalar_array_t>* timeTrajectoriesStockPtr(nullptr);
	const state_vector_array2_t* stateTrajectoriesStockPtr(nullptr);
	const input_vector_array2_t* inputTrajectoriesStockPtr(nullptr);
	ocs2QuadrupedInterfacePtr_->getOptimizedTrajectoriesPtr(
			timeTrajectoriesStockPtr,
			stateTrajectoriesStockPtr,
			inputTrajectoriesStockPtr);

	// Switching times and motion sequence
	const scalar_array_t* eventTimesPtr(nullptr);
	ocs2QuadrupedInterfacePtr_->getEventTimesPtr(eventTimesPtr);
	const size_array_t* phaseSequencePtr(nullptr);
	ocs2QuadrupedInterfacePtr_->getSubsystemsSequencePtr(phaseSequencePtr);

	// messure the delay for sending ROS messages
	if(mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_){
		finalTimePoint_ = std::chrono::steady_clock::now();
		currentDelay_ = std::chrono::duration<scalar_t, std::milli>(finalTimePoint_-startTimePoint_).count();
		meanDelay_ += (currentDelay_-meanDelay_) / numIterations_;
		maxDelay_   = std::max(maxDelay_, currentDelay_);
	}

	// messure the delay for sending ROS messages
	if(mpcSettings_.adaptiveRosMsgTimeWindow_==true)
		currentDelay_ = std::min(currentDelay_, meanDelay_*0.9);
	else
		currentDelay_ = 0.0;

	// display
	if(mpcSettings_.debugPrint_){
		std::cerr << std::endl;
		std::cerr << "### Average duration of MPC optimization is: " << meanDelay_ << " [ms]." << std::endl;
		std::cerr << "### Maximum duration of MPC optimization is: " << maxDelay_ << " [ms]." << std::endl;
	}

#ifdef PUBLISH_DUMMY

	// publish dummy for test
	publishDummy();

#else

	// publish optimized output
	if (mpcSettings_.useFeedbackPolicy_==true) {
		publishSlqController(currentTime, rbdCurrentState, controllerIsUpdated,
				controllersStockPtr,
				eventTimesPtr, phaseSequencePtr);
	} else {
		publishOptimizeTrajectories(currentTime, rbdCurrentState, controllerIsUpdated,
				timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr,
				eventTimesPtr, phaseSequencePtr);
	}

#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::targetPoseCallback(const geometry_msgs::Pose& basePoseMsg) {

	targetPositionIsUpdated_ = true;

	Eigen::Quaternion<scalar_t> qxyz(
			basePoseMsg.orientation.w,
			basePoseMsg.orientation.x,
			basePoseMsg.orientation.y,
			basePoseMsg.orientation.z);

	targetPoseDisplacement_.template head<3>() = qxyz.toRotationMatrix().eulerAngles(0, 1, 2);
	targetPoseDisplacement_(3) = basePoseMsg.position.x;
	targetPoseDisplacement_(4) = basePoseMsg.position.y;
	targetPoseDisplacement_(5) = basePoseMsg.position.z;

	// x direction
	size_t numReqiredStepsX = std::ceil( std::abs(targetPoseDisplacement_(3)) / (1.0*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// y direction
	size_t numReqiredStepsY = std::ceil( std::abs(targetPoseDisplacement_(4)) / (0.5*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// max
	size_t numReqiredSteps = std::max(numReqiredStepsX, numReqiredStepsY);

	targetReachingDuration_ = numReqiredSteps * ocs2QuadrupedInterfacePtr_->numPhasesInfullGaitCycle() * ocs2QuadrupedInterfacePtr_->strideTime();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::publisherWorkerThread() {

	while(true) {

		std::unique_lock<std::mutex> lk(publishMutex_);

		msgReady_.wait(lk, [&]{return (readyToPublish_ || stopPublishing_);});

		if (stopPublishing_==true)  break;

		if (mpcSettings_.useFeedbackPolicy_==true)
			slqControllerTrajectoryMsgBuffer_ = std::move(slqControllerTrajectoryMsg_);
		else
			switchedModelTrajectoryMsgBuffer_ = std::move(switchedModelTrajectoryMsg_);
		readyToPublish_ = false;

		lk.unlock();

		// Send data back to main()
		if (mpcSettings_.useFeedbackPolicy_==true)
			slqControllerPublisher_.publish(slqControllerTrajectoryMsgBuffer_);
		else
			slqTrajectoriesPublisher_.publish(switchedModelTrajectoryMsgBuffer_);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::launchNodes(int argc, char* argv[]) {

	// reset counters and variables
	reset();

	// display
	ROS_INFO_STREAM("MPC node is setting up ...");

	// setup ROS
	::ros::init(argc, argv, robotName_+"_mpc", ::ros::init_options::NoSigintHandler);
	signal(SIGINT, OCS2QuadrupedMPC::sigintHandler);
	::ros::NodeHandle nodeHandler;

	// SLQ-MPC subscribers
	stateSubscriber_ = nodeHandler.subscribe(robotName_+"_state", 1, &OCS2QuadrupedMPC::stateCallback, this);
	targetPoseSubscriber_ = nodeHandler.subscribe(robotName_+"_target", 1, &OCS2QuadrupedMPC::targetPoseCallback, this);

	// SLQ-MPC publishers
	if (mpcSettings_.useFeedbackPolicy_==true)
		slqControllerPublisher_ = nodeHandler.advertise<ocs2_ros_msg::slq_controller_trajectory>(robotName_+"_slq_ctlr", 1, true);
	else
		slqTrajectoriesPublisher_ = nodeHandler.advertise<ocs2_ros_msg::switched_model_trajectory>(robotName_+"_slq_trjc", 1, true);

	// dummy publisher
	dummyPublisher_ = nodeHandler.advertise<ocs2_ros_msg::dummy>("ping", 1, true);

	// display
#ifdef PUBLISH_THREAD
	std::cerr << "Publishing SLQ-MPC messages on a sparate thread." << std::endl;
#endif

	ROS_INFO_STREAM("MPC node is ready.");
	std::cerr << "Start spinning now ... " << std::endl;

	::ros::spin();
}


} // end of namespace switched_model


