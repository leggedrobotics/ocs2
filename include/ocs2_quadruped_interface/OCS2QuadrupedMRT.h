/*
 * OCS2QuadrupedMRT.h
 *
 *  Created on: Mar 7, 2018
 *      Author: farbod
 */

#ifndef OCS2QUADRUPEDMRT_H_
#define OCS2QUADRUPEDMRT_H_

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
#include <ocs2_ros_msg/gait_sequence.h>
#include <ocs2_ros_msg/switched_model_trajectory.h>
#include <ocs2_ros_msg/slq_controller_trajectory.h>
#include <ocs2_ros_msg/dummy.h>

#include <c_switched_model_interface/misc/CubicSpline.h>
#include <c_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include "c_switched_model_interface/foot_planner/FeetZDirectionPlanner.h"
#include <c_switched_model_interface/logic/SwitchedModelLogicRulesBase.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

//#define PUBLISH_THREAD

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class OCS2QuadrupedMRT
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2QuadrupedMRT<JOINT_COORD_SIZE>> Ptr;

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
	typedef typename quadruped_interface_t::control_feedback_t 	   	control_feedback_t;
	typedef typename quadruped_interface_t::controller_t			controller_t;
	typedef typename quadruped_interface_t::controller_array_t		controller_array_t;

	typedef FeetZDirectionPlanner<scalar_t,SplineCPG<scalar_t>>	feet_z_planner_t;
	typedef typename feet_z_planner_t::Ptr						feet_z_planner_ptr_t;

	// The base class for SplineCPG which is the return type of SwitchedModelPlannerLogicRules::getMotionPhaseLogics.
	typedef CPG_BASE<scalar_t>				cpg_t;
	typedef typename cpg_t::Ptr				cpg_ptr_t;
	typedef CubicSpline<scalar_t>			cubic_spline_t;
	typedef typename cubic_spline_t::Ptr	cubic_spline_ptr_t;

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE> logic_rules_t;
	typedef typename logic_rules_t::Ptr						 logic_rules_ptr_t;

	typedef ocs2::HybridLogicRulesMachine<STATE_DIM,INPUT_DIM,logic_rules_t> 	logic_machine_t;
	typedef typename logic_machine_t::Ptr 										logic_machine_ptr_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t,4>	vector_3d_array_t;

	OCS2QuadrupedMRT() = delete;

	OCS2QuadrupedMRT(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	~OCS2QuadrupedMRT();

	/**
	 * Reset flags.
	 */
	void reset();

	/**
	 * Whether the initial Plan has been already received.
	 */
	bool initialPlanReceived() const;

	/**
	 * Launches the Ros nodes to communicate with the MPC node.
	 *
	 * @param argc: command line number of inputs.
	 * @param argv: command line inputs' value.
	 */
	void launchNodes(int argc, char* argv[]);

	/**
	 * Sends the current state and checks for a new MPC update on a separate thread
	 * without blocking the main thread.
	 *
	 * @param [in] contactFlag: Current contact flag.
	 * @param [in] time: Current time.
	 * @param [in] rbdState: Current robot's RBD state
	 * @param [in] mcLoopFrequency: Motion control loop frequency (default 250 Hz)
	 */
	void updateNodes(
			const contact_flag_t& contactFlag,
			const scalar_t& time,
			const rbd_state_vector_t& rbdState,
			const scalar_t& mcLoopFrequency = 250);

	/**
	 * Computes the optimized plan for the given time based on the latest received optimized trajectory message.
	 *
	 * @param [in] time: inquiry time.
	 * @param [out] o_feetPositionRef: Planned feet positions in the origin frame.
	 * @param [out] o_feetVelocityRef: Planned feet velocities in the origin frame.
	 * @param [out] o_feetAccelerationRef: Planned feet acceleration in the origin frame.
	 * @param [out] comPoseRef: Planned CoM pose in the origin frame.
	 * @param [out] comVelocityRef: Planned CoM velocity in the origin frame.
	 * @param [out] comAccelerationRef: Planned CoM acceleration in the origin frame.
	 * @param [out] stanceLegs: Planned stance legs.
	 */
	void computePlan(
			const scalar_t& time,
			vector_3d_array_t& o_feetPositionRef,
			vector_3d_array_t& o_feetVelocityRef,
			vector_3d_array_t& o_feetAccelerationRef,
			base_coordinate_t& o_comPoseRef,
			base_coordinate_t& o_comVelocityRef,
			base_coordinate_t& o_comAccelerationRef,
			contact_flag_t& stanceLegs);

	/**
	 * Gets the swiched model plan
	 *
	 * @param [in] stateRef: state
	 * @param [in] inputRef: input
	 */
	void getComputedReferences(
			state_vector_t& stateRef,
			input_vector_t& inputRef) const;

protected:
	/**
	 * Sigint handler for ROS.
	 * @param sig
	 */
	static void sigintHandler(int sig);

	/**
	 * Publishes the current time and RBD state.
	 *
	 * @param contactFlag: Current contact flag.
	 * @param time: Current time.
	 * @param rbdState: Current robot's RBD state
	 */
	void publishCurrentState(
			const contact_flag_t& contactFlag,
			const scalar_t& time,
			const rbd_state_vector_t& rbdState) const;

	/**
	 *
	 */
	void publishDummy();

	/**
	 * Callback method to receive the optimized state-input trajectory as well as the gait sequence specifications.
	 *
	 * @param msg: A const pointer to switched_model_trajectory message.
	 */
	void optimizedTrajectoryCallback(
			const ocs2_ros_msg::switched_model_trajectory::ConstPtr& msg);

	/**
	 * Callback method to receive the optimized polict as well as the gait sequence specifications.
	 *
	 * @param msg: A const pointer to slq_controller_trajectory message.
	 */
	void slqpControllerCallback(
			const ocs2_ros_msg::slq_controller_trajectory::ConstPtr& msg);

	/**
	 * A thread function which sends the current state and checks for a new MPC update.
	 */
	void updateNodesThread();

	/**
	 * Sends the current state and checks for a new MPC update.
	 *
	 * @param contactFlag: Current contact flag.
	 * @param time: Current time.
	 * @param rbdState: Current robot's RBD state
	 */
	void updateNodesWorker(
			const contact_flag_t& contactFlag,
			const scalar_t& time,
			const rbd_state_vector_t& rbdState);

	/**
	 * Extracts eventTimes, phaseSequence from the message.
	 *
	 * @param [in] gaitSequenceMsg: Message contaning gait information.
	 * @param [out] eventTimes: Event times.
	 * @param [out] phaseSequence: Motion phase sequence.
	 * @return Whether the gait specific settings are changed from the last message.
	 */
	template <class ContainerAllocator>
	bool extractMotionSequence(
			const ocs2_ros_msg::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
			scalar_array_t& eventTimes,
			size_array_t& phaseSequence) const;

	/**
	 * Updates the internal logicRuls and logicMachine.
	 *
	 * @param [in] eventTimes: Event times.
	 * @param [in] phaseSequence: Motion phase sequence.
	 * @param [in] partitioningTimes: Time partitioning.
	 * @param [in] touchdownTimeStock: An array of touch-down times.
	 * @param [in] touchdownStateStock: An array of feet positions at touch-down.
	 * @param [in] touchdownInputStock: An array of feet velocities at touch-down.
	 */
	void updateLogics(
			const scalar_array_t& eventTimes,
			const size_array_t& phaseSequence,
			const scalar_array_t& partitioningTimes,
			const scalar_array_t& touchdownTimeStock,
			const state_vector_array_t& touchdownStateStock,
			const input_vector_array_t& touchdownInputStock);

	/**
	 * Computes feet's position, velocity, and acting contact force in the origin frame.
	 *
	 * @param [in] state: state vector.
	 * @param [in] input: input vector.
	 * @param [out] o_feetPosition: Feet's position in the origin frame.
	 * @param [out] o_feetVelocity: Feet's velocity in the origin frame.
	 * @param [out] o_contactForces: Feet's acting contact force in the origin frame.
	 */
	void computeFeetState(
			const state_vector_t& state,
			const input_vector_t& input,
			vector_3d_array_t& o_feetPosition,
			vector_3d_array_t& o_feetVelocity,
			vector_3d_array_t& o_contactForces);

private:
	/*
	 * Variables
	 */
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
	std::string robotName_;

	Model_Settings 		modelSettings_;
	ocs2::MPC_Settings 	mpcSettings_;

	feet_z_planner_ptr_t		feetZDirectionPlannerPtr_;
	logic_rules_ptr_t			logicRulesPtr_;
	logic_machine_ptr_t 		logicMachinePtr_;

	std::array<const cpg_t*,4> 		feetZPlanPtr_;
	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	std::vector<std::array<cubic_spline_ptr_t,4>> feetXPlanPtrStock_;
	std::vector<std::array<cubic_spline_ptr_t,4>> feetYPlanPtrStock_;

	scalar_t timeStep_;

	bool planReceivedEver_;
	bool planRecived_;
	bool replanMRT_;
	bool timeIsFreezed_;

	ros::Publisher  dummyPublisher_;
	ros::Publisher  statePublisher_;
	ros::Subscriber slqTrajectoriesSubscriber_;
	ros::Subscriber slqControllerSubscriber_;

	scalar_t 				planInitTime_;
	rbd_state_vector_t 		planInitState_;

	scalar_array_t 			eventTimes_;
	size_array_t 			phaseSequence_;
	scalar_array_t 			partitioningTimes_;

	scalar_array_t			timeTrajectory_;
	state_vector_array_t 	stateTrajectory_;
	input_vector_array_t 	inputTrajectory_;
	controller_t 			slqController_;

	scalar_array_t			touchdownTimeStock_;
	state_vector_array_t 	touchdownStateStock_;
	input_vector_array_t 	touchdownInputStock_;

	ocs2::LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > 		linInterpolateState_;
	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateInput_;
//	ocs2::LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > 		linInterpolateStateDerivative_;
//	ocs2::LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > 		linInterpolateUff_;
//	ocs2::LinearInterpolation<control_feedback_t, Eigen::aligned_allocator<control_feedback_t>> linInterpolateK_;

	// MPC update guard
	std::mutex optimizedTrajectoryMutex_;

	// Multi-threading for updating ROS nodes
	bool terminateThread_;
	bool readyToUpdate_;
	std::mutex updateNodesMutex_;
	std::condition_variable updateReady_;
	std::thread updateNodesThread_;

	// publishing data
	contact_flag_t currentContactFlag_;
	contact_flag_t currentContactFlagBuffer_;
	scalar_t currentTime_;
	scalar_t currentTimeBuffer_;
	rbd_state_vector_t currentRbdState_;
	rbd_state_vector_t currentRbdStateBuffer_;

	state_vector_t stateRef_;
	input_vector_t inputRef_;

};

} // end of namespace switched_model

#include "implementation/OCS2QuadrupedMRT.h"


#endif /* OCS2QUADRUPEDMRT_H_ */

