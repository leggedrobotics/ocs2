/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::MPC_ROS_Interface(
		mpc_t& mpc,
		const std::string& robotName /*= "robot"*/)

	: mpcPtr_(&mpc)
	, mpcSettings_(mpc.settings())
	, robotName_(robotName)
	, desiredTrajectoriesUpdated_(false)
{
	// correcting rosMsgTimeWindow
	if (mpcSettings_.recedingHorizon_==false)
		mpcSettings_.rosMsgTimeWindow_ = 1e+6;

	// reset variables
	reset();

	// Start thread for publishing
#ifdef PUBLISH_THREAD
	publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::~MPC_ROS_Interface() {

	shutdownNodes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::set(
		mpc_t& mpc,
		const std::string& robotName /*= "robot"*/) {

	mpcPtr_ = &mpc;
	mpcSettings_ = mpc.settings();
	robotName_ = robotName;

	// correcting rosMsgTimeWindow
	if (mpcSettings_.recedingHorizon_==false)
		mpcSettings_.rosMsgTimeWindow_ = 1e+6;

	// reset variables
	reset();
	resetRequested_ = false;

	// Start thread for publishing
#ifdef PUBLISH_THREAD
	publisherWorker_ = std::thread(&MPC_ROS_Interface::publisherWorkerThread, this);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::sigintHandler(int sig)  {

	ROS_INFO_STREAM("Shutting MPC node.");
	::ros::shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::reset() {

	initialCall_ = true;
	numIterations_ = 0;

	maxDelay_ = -1e+6;
	meanDelay_ = 0.0;
	currentDelay_ = 0.0;

	terminateThread_ = false;
	readyToPublish_  = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
bool MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::resetMpcCallback(
		ocs2_comm_interfaces::reset::Request  &req,
		ocs2_comm_interfaces::reset::Response &res) {

	if (req.reset == true) {
		// this set initialCall_ to true which invokes MPC reset at the next observation message
		resetRequested_ = true;
		reset();
		resetRequested_ = false;

		std::cerr << std::endl
				<< "\n#####################################################"
				<< "\n#####################################################"
				<< "\n#################  MPC is reset.  ###################"
				<< "\n#####################################################"
				<< "\n#####################################################"
				<< std::endl;

	} else {
		ROS_WARN_STREAM("Ineffective reset request.");
	}

	return true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publishDummy() {

	ocs2_comm_interfaces::dummy msg;
	msg.ping = 1;
	dummyPublisher_.publish(msg);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publishFeedforwardPolicy(
		const system_observation_t& currentObservation,
		const bool& controllerIsUpdated,
		const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr,
		const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
		const state_vector_array2_t*& stateTrajectoriesStockPtr,
		const input_vector_array2_t*& inputTrajectoriesStockPtr,
		const scalar_array_t*& eventTimesPtr,
		const size_array_t*& subsystemsSequencePtr)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

	ros_msg_conversions_t::CreateObservationMsg(currentObservation,
			mpcFeedforwardPolicyMsg_.initObservation);

	mpcFeedforwardPolicyMsg_.controllerIsUpdated  = controllerIsUpdated;

	ros_msg_conversions_t::CreateTargetTrajectoriesMsg(*costDesiredTrajectoriesPtr,
			mpcFeedforwardPolicyMsg_.planTargetTrajectories);

	ros_msg_conversions_t::CreateModeSequenceMsg(*eventTimesPtr, *subsystemsSequencePtr,
			mpcFeedforwardPolicyMsg_.modeSequence);

	// maximum length of the message
	size_t I = timeTrajectoriesStockPtr->size();
	size_t totalN = 0;
	for (size_t i=0; i<I; i++)
		totalN += timeTrajectoriesStockPtr->at(i).size();

	mpcFeedforwardPolicyMsg_.timeTrajectory.clear();
	mpcFeedforwardPolicyMsg_.timeTrajectory.reserve(totalN);
	mpcFeedforwardPolicyMsg_.stateTrajectory.clear();
	mpcFeedforwardPolicyMsg_.stateTrajectory.reserve(totalN);
	mpcFeedforwardPolicyMsg_.inputTrajectory.clear();
	mpcFeedforwardPolicyMsg_.inputTrajectory.reserve(totalN);

	ocs2_comm_interfaces::mpc_state mpcState;
	mpcState.value.resize(STATE_DIM);
	ocs2_comm_interfaces::mpc_input mpcInput;
	mpcInput.value.resize(INPUT_DIM);

	// The message truncation time
	const scalar_t t0 = currentObservation.time() + currentDelay_*1e-3;
	const scalar_t tf = currentObservation.time() + mpcSettings_.rosMsgTimeWindow_*1e-3;
	if (tf < t0+2.0*meanDelay_*1e-3)
		std::cout << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;

	for (size_t i=0; i<I; i++)  {

		const scalar_array_t& timeTrajectory        = timeTrajectoriesStockPtr->at(i);
		const state_vector_array_t& stateTrajectory = stateTrajectoriesStockPtr->at(i);
		const input_vector_array_t& inputTrajectory = inputTrajectoriesStockPtr->at(i);

		size_t N = timeTrajectory.size();
		if (N == 0)  continue;
		if (timeTrajectory.back()  < t0)  continue;
		if (timeTrajectory.front() > tf)  continue;

		for (size_t k=0; k<N; k++) {
			// continue if elapsed time is smaller than computation time delay
			if (k<N-1 && timeTrajectory[k+1]<t0)  continue;
			// break if the time exceed rosMsgTimeWindow
			if (timeTrajectory[k]>tf)  break;

			for (size_t j=0; j<STATE_DIM; j++)
				mpcState.value[j] = stateTrajectory[k](j);

			for (size_t j=0; j<INPUT_DIM; j++)
				mpcInput.value[j] = inputTrajectory[k](j);

			mpcFeedforwardPolicyMsg_.timeTrajectory.push_back(timeTrajectory[k]);
			mpcFeedforwardPolicyMsg_.stateTrajectory.push_back(mpcState);
			mpcFeedforwardPolicyMsg_.inputTrajectory.push_back(mpcInput);

		}  // end of k loop
	}  // end of i loop

#ifdef PUBLISH_THREAD
	readyToPublish_ = true;
	lk.unlock();
	msgReady_.notify_one();
#else
	mpcFeedforwardPolicyPublisher_.publish(mpcFeedforwardPolicyMsg_);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publishFeedbackPolicy(
		const system_observation_t& currentObservation,
		const bool& controllerIsUpdated,
		const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr,
		const controller_array_t*& controllerStockPtr,
		const scalar_array_t*& eventTimesPtr,
		const size_array_t*& subsystemsSequencePtr)  {

#ifdef PUBLISH_THREAD
	std::unique_lock<std::mutex> lk(publisherMutex_);
#endif

	ros_msg_conversions_t::CreateObservationMsg(currentObservation,
			mpcFeedbackPolicyMsg_.initObservation);

	mpcFeedbackPolicyMsg_.controllerIsUpdated  = controllerIsUpdated;

	ros_msg_conversions_t::CreateTargetTrajectoriesMsg(*costDesiredTrajectoriesPtr,
			mpcFeedbackPolicyMsg_.planTargetTrajectories);

	ros_msg_conversions_t::CreateModeSequenceMsg(*eventTimesPtr, *subsystemsSequencePtr,
			mpcFeedbackPolicyMsg_.modeSequence);

	mpcFeedbackPolicyMsg_.dimInput = INPUT_DIM;
	mpcFeedbackPolicyMsg_.dimState = STATE_DIM;

	// maximum length of the message
	size_t I = controllerStockPtr->size();
	size_t totalN = 0;
	for (size_t i=0; i<I; i++)
		totalN += controllerStockPtr->at(i).time_.size();

	mpcFeedbackPolicyMsg_.slqControllerTrajectory.clear();
	mpcFeedbackPolicyMsg_.slqControllerTrajectory.reserve(totalN);

	ocs2_comm_interfaces::slq_controller slqControllerMsg;
	slqControllerMsg.uff.value.resize(INPUT_DIM);
	slqControllerMsg.gainMatrix.resize(INPUT_DIM*STATE_DIM);

	const scalar_t t0 = currentObservation.time() + currentDelay_*1e-3;
	const scalar_t tf = currentObservation.time() + mpcSettings_.rosMsgTimeWindow_*1e-3;
	if (tf < t0+2.0*meanDelay_*1e-3)
		std::cout << "WARNING: Message publishing time-horizon is shorter than the MPC delay!" << std::endl;

	for (size_t i=0; i<I; i++)  {

		const controller_t& controller = controllerStockPtr->at(i);

		size_t N = controller.time_.size();

		if (N == 0)  continue;
		if (controller.time_.back()  < t0)  continue;
		if (controller.time_.front() > tf)  continue;

		for (size_t k=0; k<N; k++) {
			// continue if elapsed time is smaller than computation time delay
			if (k<N-1 && controller.time_[k+1]<t0)  continue;
			// break if the time exceed rosMsgTimeWindow
			if (controller.time_[k]>tf)  break;

			slqControllerMsg.timeStamp = controller.time_[k];

			for (size_t p=0; p<INPUT_DIM; p++) {
				slqControllerMsg.uff.value[p] = controller.uff_[k](p);
				for (size_t q=0; q<STATE_DIM; q++)
					slqControllerMsg.gainMatrix[p*STATE_DIM + q] = controller.k_[k](p,q);
			} // end of p loop

			mpcFeedbackPolicyMsg_.slqControllerTrajectory.push_back(slqControllerMsg);

		}  // end of k loop

	}  // end of i loop

#ifdef PUBLISH_THREAD
	readyToPublish_ = true;
	lk.unlock();
	msgReady_.notify_one();
#else
	mpcFeedbackPolicyPublisher_.publish(mpcFeedbackPolicyMsg_);
#endif

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::publisherWorkerThread() {

	while(terminateThread_==false) {

		std::unique_lock<std::mutex> lk(publisherMutex_);

		msgReady_.wait(lk, [&]{ return (readyToPublish_ || terminateThread_); });

		if (terminateThread_==true)  break;

		if (mpcSettings_.useFeedbackPolicy_==true)
			mpcFeedbackPolicyMsgBuffer_ = std::move(mpcFeedbackPolicyMsg_);
		else
			mpcFeedforwardPolicyMsgBuffer_ = std::move(mpcFeedforwardPolicyMsg_);

		readyToPublish_ = false;

		lk.unlock();
		msgReady_.notify_one();

		// publish the message
		if (mpcSettings_.useFeedbackPolicy_==true)
			mpcFeedbackPolicyPublisher_.publish(mpcFeedbackPolicyMsgBuffer_);
		else
			mpcFeedforwardPolicyPublisher_.publish(mpcFeedforwardPolicyMsgBuffer_);
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcObservationCallback(
		const ocs2_comm_interfaces::mpc_observation::ConstPtr& msg) {

	// resetting is requested
	if(resetRequested_ == true)
		return;

	// current time, state, input, and subsystem
	system_observation_t currentObservation;
	ros_msg_conversions_t::ReadObservationMsg(*msg, currentObservation);

	if (mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_)
		startTimePoint_ = std::chrono::steady_clock::now();

	// number of iterations
	numIterations_++;

	if (initialCall_==true) {
		// reset the MPC solver since it is the beginning of the task
		mpcPtr_->reset();

		// after each reset, perform user defined operation if specialized
		initCall(currentObservation);

		//** set the goal **//
		// default initial goal
		initGoalState(currentObservation, defaultCostDesiredTrajectories_);

		// display
		std::cerr << "### The target position is updated at time "
				<< std::setprecision(4) << currentObservation.time() << " as " << std::endl;
		defaultCostDesiredTrajectories_.display();

		// set CostDesiredTrajectories
		mpcPtr_->swapCostDesiredTrajectories(defaultCostDesiredTrajectories_);
	}

	// update the desired trajectories
	if(desiredTrajectoriesUpdated_==true) {

		// user defined modification of the CostDesiredTrajectories at the moment o setting
		adjustTargetTrajectories(currentObservation, costDesiredTrajectories_);

		// display
		std::cerr << "### The target position is updated at time "
				<< std::setprecision(4) << currentObservation.time() << " as " << std::endl;
		costDesiredTrajectories_.display();

		// set CostDesiredTrajectories
		mpcPtr_->swapCostDesiredTrajectories(costDesiredTrajectories_);

		desiredTrajectoriesUpdated_ = false;

	} else if (mpcSettings_.recedingHorizon_==false) {
		return;
	}

	// run SLQ-MPC
	bool controllerIsUpdated = mpcPtr_->run(
			currentObservation.time(),
			currentObservation.state());

	// get a pointer to the optimized controller
	const controller_array_t* controllersStockPtr(nullptr);
	mpcPtr_->getOptimizedControllerPtr(controllersStockPtr);
	// get a pointer to the optimized trajectories
	const std::vector<scalar_array_t>* timeTrajectoriesStockPtr(nullptr);
	const state_vector_array2_t* stateTrajectoriesStockPtr(nullptr);
	const input_vector_array2_t* inputTrajectoriesStockPtr(nullptr);
	mpcPtr_->getOptimizedTrajectoriesPtr(
			timeTrajectoriesStockPtr,
			stateTrajectoriesStockPtr,
			inputTrajectoriesStockPtr);

	// get a pointer to CostDesiredTrajectories
	const cost_desired_trajectories_t* costDesiredTrajectoriesPtr;
	mpcPtr_->getCostDesiredTrajectoriesPtr(costDesiredTrajectoriesPtr);

	// get a pointer to event times and motion sequence
	const scalar_array_t* eventTimesPtr(nullptr);
	eventTimesPtr = &mpcPtr_->getLogicRules().eventTimes();
	const size_array_t* subsystemsSequencePtr(nullptr);
	subsystemsSequencePtr = &mpcPtr_->getLogicRules().subsystemsSequence();

	// measure the delay for sending ROS messages
	if(mpcSettings_.adaptiveRosMsgTimeWindow_==true || mpcSettings_.debugPrint_){
		finalTimePoint_ = std::chrono::steady_clock::now();
		currentDelay_ = std::chrono::duration<scalar_t, std::milli>(finalTimePoint_-startTimePoint_).count();
		meanDelay_ += (currentDelay_-meanDelay_) / numIterations_;
		maxDelay_   = std::max(maxDelay_, currentDelay_);
	}

	// measure the delay for sending ROS messages
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
		publishFeedbackPolicy(currentObservation, controllerIsUpdated,
				costDesiredTrajectoriesPtr,
				controllersStockPtr,
				eventTimesPtr, subsystemsSequencePtr);
	} else {
		publishFeedforwardPolicy(currentObservation, controllerIsUpdated,
				costDesiredTrajectoriesPtr,
				timeTrajectoriesStockPtr, stateTrajectoriesStockPtr, inputTrajectoriesStockPtr,
				eventTimesPtr, subsystemsSequencePtr);
	}

#endif

	// set the initialCall flag to false
	if (initialCall_==true)
		initialCall_ = false;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::mpcTargetTrajectoriesCallback(
		const ocs2_comm_interfaces::mpc_target_trajectories::ConstPtr& msg) {

	if (desiredTrajectoriesUpdated_==false) {
		RosMsgConversions<0, 0>::ReadTargetTrajectoriesMsg(*msg, costDesiredTrajectories_);
		desiredTrajectoriesUpdated_ = true;
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::shutdownNodes() {

#ifdef PUBLISH_THREAD
	ROS_INFO_STREAM("Shutting down workers ...");

	std::unique_lock<std::mutex> lk(publisherMutex_);
	terminateThread_ = true;
	lk.unlock();

	msgReady_.notify_all();

	if (publisherWorker_.joinable())
		publisherWorker_.join();

	ROS_INFO_STREAM("All workers are shut down.");
#endif

	// shutdown publishers
	mpcFeedforwardPolicyPublisher_.shutdown();
	mpcFeedbackPolicyPublisher_.shutdown();
	dummyPublisher_.shutdown();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T>
void MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>::launchNodes(int argc, char* argv[]) {

	// reset counters and variables
	reset();

	// display
	ROS_INFO_STREAM("MPC node is setting up ...");

	// setup ROS
	::ros::init(argc, argv, robotName_+"_mpc", ::ros::init_options::NoSigintHandler);
	signal(SIGINT, MPC_ROS_Interface::sigintHandler);
	::ros::NodeHandle nodeHandler;

	// Observation subscriber
	mpcObservationSubscriber_ = nodeHandler.subscribe(
			robotName_+"_mpc_observation", 1, &MPC_ROS_Interface::mpcObservationCallback, this);

	// Goal subscriber
	mpcTargetTrajectoriesSubscriber_ = nodeHandler.subscribe(
			robotName_+"_mpc_target", 1, &MPC_ROS_Interface::mpcTargetTrajectoriesCallback, this);

	// SLQ-MPC publisher
	if (mpcSettings_.useFeedbackPolicy_==true) {
		mpcFeedbackPolicyPublisher_ = nodeHandler.advertise<ocs2_comm_interfaces::mpc_feedback_policy>(
				robotName_+"_mpc_fb_policy", 1, true);
	} else {
		mpcFeedforwardPolicyPublisher_ = nodeHandler.advertise<ocs2_comm_interfaces::mpc_feedforward_policy>(
				robotName_+"_mpc_ff_policy", 1, true);
	}

	// dummy publisher
	dummyPublisher_ = nodeHandler.advertise<ocs2_comm_interfaces::dummy>("ping", 1, true);

	// MPC reset service server
	mpcResetServiceServer_ = nodeHandler.advertiseService(robotName_+"_mpc_reset",
			&MPC_ROS_Interface::resetMpcCallback, this);

	// display
#ifdef PUBLISH_THREAD
	ROS_INFO_STREAM("Publishing SLQ-MPC messages on a separate thread.");
#endif

	ROS_INFO_STREAM("MPC node is ready.");
	ROS_INFO_STREAM("Start spinning now ...");

	::ros::spin();
}


} // namespace ocs2
