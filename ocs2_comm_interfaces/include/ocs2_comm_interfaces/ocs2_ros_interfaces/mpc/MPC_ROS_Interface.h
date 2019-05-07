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

#ifndef MPC_ROS_INTERFACE_OCS2_H_
#define MPC_ROS_INTERFACE_OCS2_H_

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
#include <ros/transport_hints.h>
#include <ros/callback_queue.h>

#include <ocs2_mpc/MPC_BASE.h>

// MPC messages
#include <ocs2_comm_interfaces/mode_sequence.h>
#include <ocs2_comm_interfaces/mpc_observation.h>
#include <ocs2_comm_interfaces/mpc_flattened_controller.h>
#include <ocs2_comm_interfaces/mpc_target_trajectories.h>
#include <ocs2_comm_interfaces/dummy.h>
#include <ocs2_comm_interfaces/reset.h>

#include "ocs2_comm_interfaces/SystemObservation.h"
#include "ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h"

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

//#define PUBLISH_DUMMY
#define PUBLISH_THREAD

namespace ocs2 {

/**
 * This class implements MPC communication interface using ROS.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class MPC_ROS_Interface
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MPC_ROS_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

	typedef MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> mpc_t;

	typedef typename mpc_t::scalar_t                   scalar_t;
	typedef typename mpc_t::scalar_array_t             scalar_array_t;
	typedef typename mpc_t::size_array_t               size_array_t;
	typedef typename mpc_t::state_vector_t             state_vector_t;
	typedef typename mpc_t::state_vector_array_t       state_vector_array_t;
	typedef typename mpc_t::state_vector_array2_t      state_vector_array2_t;
	typedef typename mpc_t::input_vector_t             input_vector_t;
	typedef typename mpc_t::input_vector_array_t       input_vector_array_t;
	typedef typename mpc_t::input_vector_array2_t      input_vector_array2_t;
	typedef typename mpc_t::input_state_matrix_t       input_state_matrix_t;
	typedef typename mpc_t::input_state_matrix_array_t input_state_matrix_array_t;

	typedef typename mpc_t::cost_desired_trajectories_t  cost_desired_trajectories_t;
	typedef typename mpc_t::mode_sequence_template_t     mode_sequence_template_t;

	typedef SystemObservation<STATE_DIM, INPUT_DIM> system_observation_t;

  typedef Controller<STATE_DIM, INPUT_DIM> controller_t;
  typedef std::vector<controller_t*> controller_ptr_array_t;

	typedef RosMsgConversions<STATE_DIM, INPUT_DIM> ros_msg_conversions_t;

	/**
	 * Default constructor
	 */
	MPC_ROS_Interface() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpc: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	MPC_ROS_Interface(
			mpc_t& mpc,
			const std::string& robotName = "robot");

	/**
	 * Destructor.
	 */
	virtual ~MPC_ROS_Interface();

	/**
	 * Sets the class as its constructor.
	 *
	 * @param [in] mpcPtr: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	void set(mpc_t& mpc,
			const std::string& robotName = "robot");

	/**
	 * Resets the class to its instantiate state.
	 */
	virtual void reset();

	/**
	 * Shutdowns the ROS node.
	 */
	void shutdownNode();

	/**
	 * Initialize the ROS node.
	 *
	 * @param [in] argc: Command line number of arguments.
	 * @param [in] argv: Command line vector of arguments.
	 */
	void initializeNode(int argc, char* argv[]);

	/**
	 * Returns a shared pointer to the node handle.
	 *
	 * @return shared pointer to the node handle.
	 */
	std::shared_ptr<ros::NodeHandle>& nodeHandlePtr();

	/**
	 * Spins ROS.
	 */
	void spin();

	/**
	 * This is the main routine which launches all the nodes required for MPC to run which includes:
	 * (1) The MPC policy publisher (either feedback or feedforward policy).
	 * (2) The observation subscriber which gets the current measured state to invoke the MPC run routine.
	 * (3) The desired trajectories subscriber which gets the goal information from user.
	 * (4) The desired mode sequence which gets the predefined mode switches for time-triggered hybrid systems.
	 *
	 * @param [in] argc: Command line number of arguments.
	 * @param [in] argv: Command line vector of arguments.
	 */
	void launchNodes(int argc, char* argv[]);

	/**
	 * This method will be called either after the very fist call of the class or after a call to reset().
	 * Users can use this function for any sort of initialization that they may need in the first call.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 */
	virtual void initCall(
			const system_observation_t& initObservation) {}

	/**
	 * Provides the initial target trajectories for the cost function.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 * @param [out] costDesiredTrajectories: The desired cost trajectories.
	 */
	virtual void initGoalState(
			const system_observation_t& initObservation,
			cost_desired_trajectories_t& costDesiredTrajectories) = 0;

	/**
	 * Provides the initial mode sequence for time-triggered hybrid systems.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 */
	virtual void initModeSequence(
			const system_observation_t& initObservation) {}

	/**
	 * Adjusts the user-defined target trajectories for the cost based on the current observation.
	 *
	 * @param [in] currentObservation: The current observation.
	 * @param costDesiredTrajectories: The received user-defined target trajectories which can be modified based on the current observation.
	 */
	virtual void adjustTargetTrajectories(
			const system_observation_t& currentObservation,
			cost_desired_trajectories_t& costDesiredTrajectories) {}

	/**
	 * Adjusts the user-defined mode sequence for time-triggered hybrid systems based on the current observation.
	 *
	 * @param [in] currentObservation: The current observation.
	 * @param newLogicRulesTemplate: New logicRules template which can be modified based on the current observation.
	 */
	virtual void adjustModeSequence(
			const system_observation_t& currentObservation,
			mode_sequence_template_t& newLogicRulesTemplate) {}

protected:
	/**
	 * Signal handler
	 *
	 * @param sig: signal
	 */
	static void sigintHandler(int sig);

	/**
	 * Callback to reset MPC.
	 *
	 * @param req: Service request.
	 * @param res: Service response.
	 */
	bool resetMpcCallback(
			ocs2_comm_interfaces::reset::Request  &req,
			ocs2_comm_interfaces::reset::Response &res);

	/**
	 * Dummy publisher for network debugging.
	 */
	void publishDummy();

	/**
	 * Publishes the MPC feedforward policy.
	 *
	 * @param [in] currentObservation: The observation that MPC designed from.
	 * @param [in] controllerIsUpdated: Whether the policy is updated.
	 * @param [in] costDesiredTrajectoriesPtr: The target trajectories that MPC optimized.
	 * @param [in] controllerStockPtr: A pointer to the MPC optimized control policy.
	 * @param [in] timeTrajectoriesStockPtr: A pointer to the MPC optimized time trajectory.
	 * @param [in] stateTrajectoriesStockPtr: A pointer to the  MPC optimized state trajectory.
	 * @param [in] inputTrajectoriesStockPtr: A pointer to the  MPC optimized input trajectory.
	 * @param [in] eventTimesPtr: A pointer to the event time sequence.
	 * @param [in] subsystemsSequencePtr: A pointer to the subsystem sequence.
	 */
	void publishPolicy(
			const system_observation_t& currentObservation,
			const bool& controllerIsUpdated,
			const cost_desired_trajectories_t*& costDesiredTrajectoriesPtr,
			const controller_ptr_array_t*& controllerStockPtr,
			const std::vector<scalar_array_t>*& timeTrajectoriesStockPtr,
			const state_vector_array2_t*& stateTrajectoriesStockPtr,
			const input_vector_array2_t*& inputTrajectoriesStockPtr,
			const scalar_array_t*& eventTimesPtr,
			const size_array_t*& subsystemsSequencePtr);

	/**
	 * Handles ROS publishing thread.
	 */
	void publisherWorkerThread();

	/**
	 * The callback method which receives the current observation, invokes the MPC algorithm,
	 * and finally publishes the optimized policy.
	 *
	 * @param [in] msg: The observation message.
	 */
	void mpcObservationCallback(
			const ocs2_comm_interfaces::mpc_observation::ConstPtr& msg);

	/**
	 * The callback method which receives the user-defined target trajectories message.
	 *
	 * @param [in] msg: The target trajectories message.
	 */
	void mpcTargetTrajectoriesCallback(
			const ocs2_comm_interfaces::mpc_target_trajectories::ConstPtr& msg);

	/**
	 * The callback method which receives the user-defined mode sequence message.
	 *
	 * @param [in] msg: The mode sequence message.
	 */
	void mpcModeSequenceCallback(
			const ocs2_comm_interfaces::mode_sequence::ConstPtr& msg);


protected:
	/*
	 * Variables
	 */
	mpc_t* mpcPtr_;
	MPC_Settings mpcSettings_;

	std::string robotName_;

	std::shared_ptr<ros::NodeHandle> nodeHandlerPtr_;

	// Publishers and subscribers
	::ros::Subscriber    mpcObservationSubscriber_;
	::ros::Subscriber    mpcTargetTrajectoriesSubscriber_;
	::ros::Subscriber    mpcModeSequenceSubscriber_;
	::ros::Publisher     mpcPolicyPublisher_;
	::ros::Publisher     dummyPublisher_;
	::ros::ServiceServer mpcResetServiceServer_;

	// MPC reset flags
	std::atomic<bool> resetRequested_;

	// ROS messages
	ocs2_comm_interfaces::mpc_flattened_controller mpcPolicyMsg_;
	ocs2_comm_interfaces::mpc_flattened_controller mpcPolicyMsgBuffer_;

	// Multi-threading for publishers
	bool terminateThread_;
	bool readyToPublish_;
	std::thread publisherWorker_;
	std::mutex publisherMutex_;
	std::condition_variable msgReady_;

	size_t numIterations_;
	scalar_t maxDelay_;
	scalar_t meanDelay_;
	scalar_t currentDelay_;

	std::chrono::time_point<std::chrono::steady_clock> startTimePoint_;
	std::chrono::time_point<std::chrono::steady_clock> finalTimePoint_;

	bool initialCall_;

	std::atomic<bool> desiredTrajectoriesUpdated_;
	std::atomic<bool> modeSequenceUpdated_;
	cost_desired_trajectories_t costDesiredTrajectories_;
	cost_desired_trajectories_t defaultCostDesiredTrajectories_;
	mode_sequence_template_t modeSequenceTemplate_;
};

} // namespace ocs2

#include "implementation/MPC_ROS_Interface.h"

#endif /* MPC_ROS_INTERFACE_OCS2_H_ */
