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

#ifndef MRT_ROS_INTERFACE_OCS2_H_
#define MRT_ROS_INTERFACE_OCS2_H_

#include <array>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/transport_hints.h>

#include <ocs2_core/Dimensions.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// MPC messages
#include <ocs2_comm_interfaces/mpc_flattened_controller.h>
#include <ocs2_comm_interfaces/dummy.h>
#include <ocs2_comm_interfaces/reset.h>

#include "ocs2_comm_interfaces/SystemObservation.h"
#include "ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h"

#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MRT (Model Reference Tracking) communication interface using ROS.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class MRT_ROS_Interface
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MRT_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef Dimensions<STATE_DIM, INPUT_DIM> DIMENSIONS;
	typedef typename DIMENSIONS::scalar_t       scalar_t;
	typedef typename DIMENSIONS::scalar_array_t	scalar_array_t;
	typedef typename DIMENSIONS::size_array_t   size_array_t;
	typedef typename DIMENSIONS::state_vector_t       state_vector_t;
	typedef typename DIMENSIONS::state_vector_array_t state_vector_array_t;
	typedef typename DIMENSIONS::input_vector_t       input_vector_t;
	typedef typename DIMENSIONS::input_vector_array_t input_vector_array_t;
	typedef typename DIMENSIONS::input_state_matrix_t       input_state_matrix_t;
	typedef typename DIMENSIONS::input_state_matrix_array_t input_state_matrix_array_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	typedef SystemObservation<STATE_DIM, INPUT_DIM> system_observation_t;

	typedef RosMsgConversions<STATE_DIM, INPUT_DIM> ros_msg_conversions_t;

	typedef HybridLogicRulesMachine<LOGIC_RULES_T> logic_machine_t;
	typedef typename logic_machine_t::Ptr          logic_machine_ptr_t;

  	typedef typename std::unique_ptr<RolloutBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> rollout_base_ptr_t;
  	typedef TimeTriggeredRollout<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> time_triggered_rollout_t;
	typedef ControlledSystemBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> controlled_system_base_t;

	typedef LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_linear_interpolation_t;

	typedef ControllerBase<STATE_DIM,INPUT_DIM> controller_t;

	/**
	 * Default constructor
	 */
	MRT_ROS_Interface() = default;

	/**
	 * Constructor
	 *
	 * @param [in] logicRules: A logic rule class of derived from the hybrid logicRules base.
	 * @param [in] robotName: The robot's name.
	 */
	MRT_ROS_Interface(
			const LOGIC_RULES_T& logicRules,
			const std::string& robotName = "robot");

	/**
	 * Destructor
	 */
	virtual ~MRT_ROS_Interface();

	/**
	 * Sets the class as its constructor.
	 *
	 * @param [in] logicRules: A logic rule class of derived from the hybrid logicRules base.
	 * @param [in] robotName: The robot's name.
	 */
	void set(const LOGIC_RULES_T& logicRules,
			const std::string& robotName = "robot");

	/**
	 * Resets the class to its instantiate state.
	 */
	virtual void reset();

	/**
	 * Request the MPC node to reset. This method is a blocking method.
	 *
	 * @param [in] initCostDesiredTrajectories: The initial desired cost trajectories.
	 */
	void resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories);

	/**
	 * This method will be called either after the very fist call of the class or after a call to reset().
	 * Users can use this function for any sort of initialization that they may need in the first call.
	 *
	 * @param [in] planObservation: The observation of the policy.
	 */
	virtual void initCall(
			const system_observation_t& planObservation) {}

	/**
	 * Whether the initial MPC policy has been already received.
	 */
	bool initialPolicyReceived() const;

	/**
	 * Gets a reference to CostDesiredTrajectories for which the current policy is optimized for.
	 *
	 * @return a constant reference to CostDesiredTrajectories of the policy.
	 */
	const cost_desired_trajectories_t& mpcCostDesiredTrajectories() const;

  	/**
	 * Initializes rollout class to roll out a feedback policy
	 *
	 * @param [in] controlSystemBasePtr: Pointer to the system to roll out.
	 * @param [in] rollout settings.
	 */
	void initRollout(
			const controlled_system_base_t& controlSystemBase,
			const Rollout_Settings& rolloutSettings);

	/**
	 * @brief Interpolates nominal state and active subsystem at given time. Does not use the controller.
	 *
	 * @param [in] currentTime: the query time.
	 * @param [in] currentState: the query state.
	 * @param [out] mpcState: the current nominal state of MPC.
	 * @param [out] mpcInput: the optimized control input.
	 * @param [out] subsystem: the active subsystem.
	 */
	void evaluatePolicy(
			const scalar_t& currentTime,
			const state_vector_t& currentState,
			state_vector_t& mpcState,
			input_vector_t& mpcInput,
			size_t& subsystem);

	/**
	 * Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
	 *
	 * @param [in] currentTime: start of the rollout.
	 * @param [in] currentState: state to start rollout from.
	 * @param [in] timeStep: duration of the forward rollout.
	 * @param [out] mpcState: the new forwarded state of MPC.
	 * @param [out] mpcInput: the new control input of MPC.
	 * @param [out] subsystem: the active subsystem.
	 */
	void rolloutPolicy(
			const scalar_t& currentTime,
			const state_vector_t& currentState,
			const scalar_t& timeStep,
			state_vector_t& mpcState,
			input_vector_t& mpcInput,
			size_t& subsystem);

	/**
	 * Shutdowns the ROS nodes.
	 */
	void shutdownNodes();

	/**
	 * spin the MRT callback queue
	 */
	void spinMRT();

	/**
	 * Launches the ROS nodes to communicate with the MPC node.
	 *
	 * @param [in] argc: Command line number of arguments.
	 * @param [in] argv: Command line vector of arguments.
	 */
	void launchNodes(int argc, char* argv[]);

	/**
	 *  Gets the node handle pointer to the MRT node,
	 *  Use this to add subscribers to the custom MRT callback queue
	 */
	::ros::NodeHandlePtr& nodeHandle();

	/**
	 * Publishes the current observation on a separate thread
	 * without blocking the main thread.
	 *
	 * @param [in] currentObservation: The current observation.
	 */
	void publishObservation(
			const system_observation_t& currentObservation);

	/**
	 * Checks the data buffer for an update of the MPC policy. If a new policy
	 * is available on the buffer this method will load it to the in-use policy.
	 * This method also calls the modifyPolicy() method.
	 *
	 * Make sure to call spinMRT() to check for new messages
	 *
	 * @return True if the policy is updated.
	 */
	bool updatePolicy();

	/**
	 * Returns if MPC has been terminated due to an exception.
	 *
	 * @return true if MPC failed.
	 */
	bool mpcIsTerminated() const;

protected:
	/**
	 * The updatePolicy() method will call this method which allows the user to
	 * customize the in-use policy. Note that this method is already
	 * protected with a mutex which blocks the policy callback. Moreover, this method
	 * may be called in the main thread of the program. Thus, for efficiency and
	 * practical considerations you should avoid computationally expensive operations.
	 * For such operations you may want to use the modifyBufferPolicy()
	 * methods which runs on a separate thread which directly modifies the received
	 * policy messages on the data buffer.
	 *
	 * @param logicUpdated: Whether eventTimes or subsystemsSequence are updated form the last call.
	 * @param mpcController: The optimized control policy of MPC.
	 * @param mpcTimeTrajectory: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectory: The optimized state trajectory of the policy message on the buffer.
	 * @param eventTimes: The event times of the policy.
	 * @param subsystemsSequence: The subsystems sequence of the policy.
	 */
	virtual void modifyPolicy(
			bool& logicUpdated,
			controller_t& mpcController,
			scalar_array_t& mpcTimeTrajectory,
			state_vector_array_t& mpcStateTrajectory,
			scalar_array_t& eventTimes,
			size_array_t& subsystemsSequence) {}

	/**
	 * This method can be used to modify the policy on the buffer without inputting the
	 * main thread. Note that the variables that are on the buffer have the suffix Buffer. It is
	 * important if any new variables are added to the policy also obey this rule. These buffer
	 * variables can be later, in the modifyPolicy() method, swept to the in-use policy memory.
	 *
	 * @param [in] mpcInitObservationBuffer: The observation of the policy message on the buffer.
	 * @param mpcControllerBuffer: The optimized controller of the policy message on the buffer.
	 * @param mpcTimeTrajectoryBuffer: The optimized time trajectory of the policy message on the buffer.
	 * @param mpcStateTrajectoryBuffer: The optimized state trajectory of the policy message on the buffer.
	 * @param eventTimesBuffer: The event times of the policy message on the buffer.
	 * @param subsystemsSequenceBuffer: The subsystems sequence of the policy message on the buffer.
	 */
	virtual void modifyBufferPolicy(
			const system_observation_t& mpcInitObservationBuffer,
			controller_t& mpcControllerBuffer,
			scalar_array_t& mpcTimeTrajectoryBuffer,
			state_vector_array_t& mpcStateTrajectoryBuffer,
			scalar_array_t& eventTimesBuffer,
			size_array_t& subsystemsSequenceBuffer) {}

	/**
	 * Signal handler
	 *
	 * @param sig: signal
	 */
	static void sigintHandler(int sig);

	/**
	 * Dummy publisher for network debugging.
	 */
	void publishDummy();

	/**
	 * Callback method to receive the MPC policy as well as the mode sequence.
	 * It only updates the policy variables with suffix (*Buffer_) variables.
	 *
	 * @param msg: A constant pointer to the message
	 */
	void mpcPolicyCallback(
			const ocs2_comm_interfaces::mpc_flattened_controller::ConstPtr& msg);

	/**
	 * A thread function which sends the current state and checks for a new MPC update.
	 */
	void publisherWorkerThread();

	/**
	 * Gets the hash value of the message based on its system_observation.
	 *
	 * @param [in] observation: The observation structure of the message.
	 * @return The message hash value.
	 */
	size_t messageHashValue(
			const system_observation_t& observation) const;

	/**
	 * Constructs a partitioningTimes vector with 2 elements: minimum of the already
	 * received times and the maximum value of the numeric type scalar_t. This prevents
	 * the frequent update of the logicRules.
	 *
	 * @param [in] time: The current time.
	 * @param [out] partitioningTimes: Partitioning time.
	 */
	void partitioningTimesUpdate(
			const scalar_t& time,
			scalar_array_t& partitioningTimes) const;

protected:
	/*
	 * Variables
	 */
	std::string robotName_;

	logic_machine_ptr_t logicMachinePtr_;

	::ros::NodeHandlePtr mrtRosNodeHandlePtr_;

	// Publishers and subscribers
	::ros::Publisher  dummyPublisher_;
	::ros::Publisher  mpcObservationPublisher_;
	::ros::Subscriber mpcPolicySubscriber_;
	::ros::ServiceClient mpcResetServiceClient_;

	// ROS messages
	ocs2_comm_interfaces::mpc_observation mpcObservationMsg_;
	ocs2_comm_interfaces::mpc_observation mpcObservationMsgBuffer_;

	// Multi-threading for subscribers
	mutable std::mutex policyMutex_;        // for policy variables WITHOUT suffix (*Buffer_) variables
	mutable std::mutex policyMutexBuffer_;  // for policy variables WITH suffix (*Buffer_) variables
	::ros::CallbackQueue mrtCallbackQueue_;

	// Multi-threading for publishers
	bool terminateThread_;
	bool readyToPublish_;
	std::thread publisherWorker_;
	std::mutex publisherMutex_;
	std::condition_variable msgReady_;

	bool logicUpdated_;
	bool policyUpdated_;
	bool policyUpdatedBuffer_;
	std::atomic<bool> policyReceivedEver_;
	system_observation_t initPlanObservation_;

	size_t messageHash_;
	size_t messageHashBuffer_;

	system_observation_t mpcInitObservation_;
	system_observation_t mpcInitObservationBuffer_;

	scalar_array_t eventTimes_;
	scalar_array_t eventTimesBuffer_;
	size_array_t   subsystemsSequence_;
	size_array_t   subsystemsSequenceBuffer_;
	scalar_array_t partitioningTimes_;
	scalar_array_t partitioningTimesBuffer_;

	std::unique_ptr<controller_t> mpcControllerPtr_;
	std::unique_ptr<controller_t> mpcControllerBufferPtr_;
	scalar_array_t       mpcTimeTrajectory_;
	scalar_array_t       mpcTimeTrajectoryBuffer_;
	state_vector_array_t mpcStateTrajectory_;
	state_vector_array_t mpcStateTrajectoryBuffer_;
	state_linear_interpolation_t mpcLinInterpolateState_;
	cost_desired_trajectories_t  mpcCostDesiredTrajectories_;
	cost_desired_trajectories_t  mpcCostDesiredTrajectoriesBuffer_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

  	rollout_base_ptr_t rolloutPtr_;
};

} // namespace ocs2

#include "implementation/MRT_ROS_Interface.h"

#endif /* MRT_ROS_INTERFACE_OCS2_H_ */
