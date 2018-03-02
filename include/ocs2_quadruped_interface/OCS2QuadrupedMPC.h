/*
 * OCS2QuadrupedMPC.h
 *
 *  Created on: Mar 1, 2018
 *      Author: farbod
 */

#ifndef OCS2QUADRUPEDMPC_H_
#define OCS2QUADRUPEDMPC_H_

#include <array>
#include <memory>
#include <vector>
#include <csignal>
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// OCS2 messages
#include <ocs2_ros_msg/rbd_state_vector.h>
#include <ocs2_ros_msg/switched_model_trajectory.h>
#include <ocs2_ros_msg/slq_controller_trajectory.h>
#include <ocs2_ros_msg/dummy.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"


//#define PUBLISH_DUMMY
#define PUBLISH_THREAD

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class OCS2QuadrupedMPC
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr quadruped_interface_ptr_t;

	enum {
		STATE_DIM = quadruped_interface_t::STATE_DIM,
		INPUT_DIM = quadruped_interface_t::INPUT_DIM,
		RBD_STATE_DIM = quadruped_interface_t::RBD_STATE_DIM
	};

	typedef typename quadruped_interface_t::com_model_t			com_model_t;
	typedef typename quadruped_interface_t::kinematic_model_t 	kinematic_model_t;
	typedef typename quadruped_interface_t::state_estimator_t	state_estimator_t;

	typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
	typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
	typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

	typedef typename quadruped_interface_t::scalar_t				scalar_t;
	typedef typename quadruped_interface_t::scalar_array_t			scalar_array_t;
	typedef typename quadruped_interface_t::size_array_t			size_array_t;
	typedef typename quadruped_interface_t::state_vector_t			state_vector_t;
	typedef typename quadruped_interface_t::state_vector_array_t	state_vector_array_t;
	typedef typename quadruped_interface_t::state_vector_array2_t	state_vector_array2_t;
	typedef typename quadruped_interface_t::input_vector_t			input_vector_t;
	typedef typename quadruped_interface_t::input_vector_array_t	input_vector_array_t;
	typedef typename quadruped_interface_t::input_vector_array2_t	input_vector_array2_t;
	typedef typename quadruped_interface_t::controller_t			controller_t;
	typedef typename quadruped_interface_t::controller_array_t		controller_array_t;

	typedef std::shared_ptr<const std::vector<scalar_array_t>> 	scalar_array2_ptr_t;
	typedef std::shared_ptr<const state_vector_array2_t> 		state_vector_array2_ptr_t;
	typedef std::shared_ptr<const input_vector_array2_t> 		input_vector_array2_ptr_t;
	typedef std::shared_ptr<const controller_array_t> 			controller_array_ptr_t;


	OCS2QuadrupedMPC(const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	virtual ~OCS2QuadrupedMPC();

	void reset();

	void publishDummy();

	void publishOptimizeTrajectories(
			const scalar_t& planInitTime,
			const rbd_state_vector_t& planInitState,
			const bool& controllerIsUpdated,
			const scalar_array2_ptr_t& timeTrajectoriesStockPtr,
			const state_vector_array2_ptr_t& stateTrajectoriesStockPtr,
			const input_vector_array2_ptr_t& inputTrajectoriesStockPtr,
			const size_array_t& switchingModeSequence,
			const scalar_array_t& switchingTimes);

	void publishSlqController(
			const scalar_t& planInitTime,
			const rbd_state_vector_t& planInitState,
			const bool& controllerIsUpdated,
			const controller_array_ptr_t& controllerTrajectoriesStock,
			const size_array_t& switchingModeSequence,
			const scalar_array_t& switchingTimes);

	void stateCallback(const ocs2_ros_msg::rbd_state_vector::ConstPtr msg);

	void targetPoseCallback(const geometry_msgs::Point& msg);

	void launchNodes(int argc, char* argv[]);

	static void sigintHandler(int sig);

	void publisherWorkerThread();

private:
	/*
	 * Variables
	 */
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
	std::string robotName_;

	ocs2::MPC_Settings mpcSettings_;

	// Publishers and subscribers
	ros::Subscriber stateSubscriber_;
	ros::Subscriber targetPoseSubscriber_;
	ros::Publisher  slqTrajectoriesPublisher_;
	ros::Publisher  slqControllerPublisher_;
	ros::Publisher  dummyPublisher_;

	// Threading for publishing data
	bool isPublish_;
	bool readyToPublish_;
	std::thread publisherWorker_;
	std::mutex publishMutex_;
	std::condition_variable msgReady_;

	bool initialCall_;
	state_vector_t initState_;

	size_t numIterations_;
	scalar_t maxDelay_;
	scalar_t meanDelay_;
	scalar_t currentDelay_;

	std::chrono::time_point<std::chrono::steady_clock> startTimePoint_;
	std::chrono::time_point<std::chrono::steady_clock> finalTimePoint_;

	bool targetPositionIsUpdated_;
	scalar_t targetReachingDuration_;
	base_coordinate_t targetPoseDisplacement_;

	// optimized outputs
	controller_array_ptr_t 		controllersStock_;
	scalar_array2_ptr_t 		timeTrajectoriesStock_;
	state_vector_array2_ptr_t 	stateTrajectoriesStock_;
	input_vector_array2_ptr_t 	inputTrajectoriesStock_;
	// gait characteristics
	size_array_t gaitSequence_;
	scalar_array_t switchingTimes_;

	// optimal outputs messages
	ocs2_ros_msg::slq_controller_trajectory slqControllerTrajectoryMsg_;
	ocs2_ros_msg::switched_model_trajectory switchedModelTrajectoryMsg_;
	ocs2_ros_msg::slq_controller_trajectory slqControllerTrajectoryMsgBuffer_;
	ocs2_ros_msg::switched_model_trajectory switchedModelTrajectoryMsgBuffer_;

};

} // end of namespace switched_model

#include "implementation/OCS2QuadrupedMPC.h"

#endif /* OCS2QUADRUPEDMPC_H_ */
