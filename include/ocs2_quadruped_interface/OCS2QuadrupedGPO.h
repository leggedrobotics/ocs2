/*
 * OCS2QuadrupedGPO.h
 *
 *  Created on: Mar 15, 2018
 *      Author: farbod
 */

#ifndef OCS2QUADRUPEDGPO_H_
#define OCS2QUADRUPEDGPO_H_

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
#include <algorithm>
#include <condition_variable>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

// OCS2 messages
#include <ocs2_ros_msg/gait_sequence.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class OCS2QuadrupedGPO
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
	typedef typename quadruped_interface_t::control_feedback_t 	   	control_feedback_t;

	typedef typename quadruped_interface_t::feet_z_planner_t		feet_z_planner_t;
	typedef typename quadruped_interface_t::feet_z_planner_ptr_t	feet_z_planner_ptr_t;
	typedef typename quadruped_interface_t::logic_rules_t			logic_rules_t;
	typedef typename quadruped_interface_t::logic_rules_ptr_t		logic_rules_ptr_t;
	typedef typename quadruped_interface_t::slq_base_ptr_t			slq_base_ptr_t;


	OCS2QuadrupedGPO(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	~OCS2QuadrupedGPO() = default;

	void reset();

	void launchNodes(int argc, char* argv[]);

	void userDefinedGaitCallback (
			const ocs2_ros_msg::gait_sequence::ConstPtr& msg);

	void run();


protected:
	/**
	 * Sigint handler for ROS.
	 * @param sig
	 */
	static void sigintHandler(int sig);

	/**
	 * Extracts eventTimes, phaseSequence from the template gait.
	 *
	 * @param [in] gaitSequenceMsg: Message contaning gait information.
	 * @param [out] eventTimes: Event times.
	 * @param [out] phaseSequence: Motion phase sequence.
	 * @return Whether the gait specific settings are changed from the last message.
	 */
	template <class ContainerAllocator>
	bool extractTemplateMotionSequence(
			const ocs2_ros_msg::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
			scalar_array_t& eventTimes,
			size_array_t& phaseSequence) const;

	scalar_t computeLogicFinalTime(
			const scalar_array_t& partitioningTimes) const;

private:
	/*
	 * Variables
	 */
	const scalar_t phaseTransitionStanceTime_ = 0.4;

	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
	std::string robotName_;

	ocs2::MPC_Settings mpcSettings_;

	const slq_base_ptr_t slqPtr_;

	unsigned long long int slqRewindCounterCopy_;

	bool userDefinedGait_;
	scalar_array_t eventTimesTemplate_;
	size_array_t phaseSequenceTemplate_;

	ros::Subscriber userDefinedGaitSubscriber_;

	scalar_t logicFinalTime_;
	scalar_array_t partitioningTimes_;

	bool logicRulesIsUpdated_;
	logic_rules_t optimizedLogicRules_;
};

} // end of namespace switched_model

#include "implementation/OCS2QuadrupedGPO.h"

#endif /* OCS2QUADRUPEDGPO_H_ */
