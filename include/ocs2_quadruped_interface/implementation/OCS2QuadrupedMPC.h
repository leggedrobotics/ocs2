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
	std::cerr << "Shutting down workers." << std::endl;

	isPublish_ = false;
	publisherWorker_.join();

	std::cerr << "All workers shut down" << std::endl;
#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::sigintHandler(int sig)  {

	std::cerr << "Shutting MPC node. " << std::endl;
	ros::shutdown();
	exit(0);
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

	isPublish_ = true;
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
		const scalar_array2_ptr_t& timeTrajectoriesStockPtr,
		const state_vector_array2_ptr_t& stateTrajectoriesStockPtr,
		const input_vector_array2_ptr_t& inputTrajectoriesStockPtr,
		const size_array_t& gaitSequence,
		const scalar_array_t& switchingTimes)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publishMutex_);
#endif

	switchedModelTrajectoryMsg_.controllerIsUpdated  = controllerIsUpdated;
	switchedModelTrajectoryMsg_.planInitTime = planInitTime;
	for (size_t i=0; i<RBD_STATE_DIM; i++)
		switchedModelTrajectoryMsg_.planInitState[i] = planInitState(i);

	switchedModelTrajectoryMsg_.timeTrajectory.clear();
	switchedModelTrajectoryMsg_.stateTrajectory.clear();
	switchedModelTrajectoryMsg_.inputTrajectory.clear();
	switchedModelTrajectoryMsg_.subsystemNum.clear();
	switchedModelTrajectoryMsg_.switchingTimes.clear();
	switchedModelTrajectoryMsg_.gaitSequence.clear();

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

		switchedModelTrajectoryMsg_.gaitSequence.push_back(gaitSequence[i]);
		switchedModelTrajectoryMsg_.switchingTimes.push_back(switchingTimes[i]);

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

			switchedModelTrajectoryMsg_.subsystemNum.push_back(i);

		}  // end of k loop
	}  // end of i loop

	switchedModelTrajectoryMsg_.switchingTimes.push_back(switchingTimes[lastActiveSubsystem+1]);

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
		const controller_array_ptr_t& controllerTrajectoriesStock,
		const size_array_t& gaitSequence,
		const scalar_array_t& switchingTimes)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publishMutex_);
#endif
	ocs2_ros_msg::slq_controller slqControllerMsg;
	bool nanFound = false;

	slqControllerTrajectoryMsg_.numInputs  = INPUT_DIM;
	slqControllerTrajectoryMsg_.numOutputs = STATE_DIM;

	slqControllerTrajectoryMsg_.controllerIsUpdated = controllerIsUpdated;
	slqControllerTrajectoryMsg_.planInitTime = planInitTime;
	for (size_t i=0; i<RBD_STATE_DIM; i++)
		slqControllerTrajectoryMsg_.planInitState[i] = planInitState(i);

	slqControllerTrajectoryMsg_.slqControllerTrajectory.clear();
	slqControllerTrajectoryMsg_.switchingTimes.clear();
	slqControllerTrajectoryMsg_.gaitSequence.clear();

	const scalar_t t0 = planInitTime + currentDelay_*1e-3;
	const scalar_t tf = planInitTime + mpcSettings_.rosMsgTimeWindow_*1e-3;
	if (tf < t0+2.0*meanDelay_*1e-3)
		std::cout << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;

	int lastActiveSubsystem = -1;
	for (size_t i=0; i<controllerTrajectoriesStock->size(); i++)  {

		const controller_t& controller = controllerTrajectoriesStock->at(i);

		size_t N = controller.time_.size();

		if (N == 0)  continue;
		if (controller.time_.back()  < t0)  continue;
		if (controller.time_.front() > tf)  continue;

		slqControllerTrajectoryMsg_.gaitSequence.push_back(gaitSequence[i]);
		slqControllerTrajectoryMsg_.switchingTimes.push_back(switchingTimes[i]);

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

			slqControllerMsg.subsystemNum = i;
		}  // end of k loop

	}  // end of i loop

	slqControllerTrajectoryMsg_.switchingTimes.push_back(switchingTimes[lastActiveSubsystem+1]);

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
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::stateCallback(const ocs2_ros_msg::rbd_state_vector::ConstPtr msg) {

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

		if (mpcSettings_.recedingHorizon_==true) {
			targetPositionIsUpdated_ = true;
			targetPoseDisplacement_.setZero();
			targetReachingDuration_ = ocs2QuadrupedInterfacePtr_->numPhasesInfullGaitCycle() * ocs2QuadrupedInterfacePtr_->strideTime();
		} else {
			targetPositionIsUpdated_ = false;
		}
	}

	// update target position
	if(targetPositionIsUpdated_==true) {

		targetPositionIsUpdated_ = false;

		state_vector_t currentState;
		ocs2QuadrupedInterfacePtr_->computeSwitchedModelState(rbdCurrentState, currentState);

		state_vector_t targetState;
		targetState.template segment<3>(0) = initState_. template segment<3>(0)   + targetPoseDisplacement_.template segment<3>(0);
		targetState.template segment<2>(3) = currentState. template segment<2>(3) + targetPoseDisplacement_.template segment<2>(3);
		targetState.template segment<1>(5) = initState_. template segment<1>(5)   + targetPoseDisplacement_.template segment<1>(5);
		targetState.template segment<6>(6).setZero();
		targetState.template segment<12>(12) = initState_.template segment<12>(12);

		ocs2QuadrupedInterfacePtr_->setNewGoalStateMPC(targetReachingDuration_, targetState);

		// display
		std::cerr << "### The target position is updated to at time " << std::setprecision(4) << currentTime << " to:    "
				<< targetState.template segment<3>(3).transpose() << std::endl;

	} else if (mpcSettings_.recedingHorizon_==false) {
		return;
	}

	// run SLQ-MPC
	bool controllerIsUpdated = ocs2QuadrupedInterfacePtr_->runMPC(currentTime, rbdCurrentState);
	// get optimized output
	controllersStock_ = ocs2QuadrupedInterfacePtr_->getControllerPtr();
	timeTrajectoriesStock_  = ocs2QuadrupedInterfacePtr_->getTimeTrajectoriesPtr();
	stateTrajectoriesStock_ = ocs2QuadrupedInterfacePtr_->getStateTrajectoriesPtr();
	inputTrajectoriesStock_ = ocs2QuadrupedInterfacePtr_->getInputTrajectoriesPtr();

	// Switching times and motion sequence
	ocs2QuadrupedInterfacePtr_->getSwitchingTimes(switchingTimes_);
	ocs2QuadrupedInterfacePtr_->getGaitSequence(gaitSequence_);

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
		publishSlqController(currentTime, rbdCurrentState, controllerIsUpdated, controllersStock_,
				gaitSequence_, switchingTimes_);
	} else {
		publishOptimizeTrajectories(currentTime, rbdCurrentState, controllerIsUpdated,
				timeTrajectoriesStock_, stateTrajectoriesStock_, inputTrajectoriesStock_,
				gaitSequence_, switchingTimes_);
	}

#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void OCS2QuadrupedMPC<JOINT_COORD_SIZE>::targetPoseCallback(const geometry_msgs::Point& msg) {

	targetPositionIsUpdated_ = true;

	targetPoseDisplacement_(0) = 0.0;
	targetPoseDisplacement_(1) = 0.0;
	targetPoseDisplacement_(2) = 0.0;
	targetPoseDisplacement_(3) = msg.x;
	targetPoseDisplacement_(4) = msg.y;
	targetPoseDisplacement_(5) = msg.z;

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

	while(isPublish_) {

		std::unique_lock<std::mutex> lk(publishMutex_);
		msgReady_.wait(lk, [&]{return readyToPublish_;});

		slqControllerTrajectoryMsgBuffer_ = slqControllerTrajectoryMsg_;
		switchedModelTrajectoryMsgBuffer_ = switchedModelTrajectoryMsg_;
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
	std::cerr << "ROS node is setting up."<< std::endl;

	// setup ROS
	ros::init(argc, argv, robotName_+"_mpc", ros::init_options::NoSigintHandler);
	signal(SIGINT, OCS2QuadrupedMPC::sigintHandler);
	ros::NodeHandle nodeHandler;

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
	std::cerr << "Start spinning now ... " << std::endl;


	ros::spin();
}


} // end of namespace switched_model


