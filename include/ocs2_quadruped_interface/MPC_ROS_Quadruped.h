/*
 * MPC_ROS_Quadruped.h
 *
 *  Created on: May 27, 2018
 *      Author: farbod
 */

#ifndef MPC_ROS_QUADRUPED_H_
#define MPC_ROS_QUADRUPED_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedInterface.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM=12+JOINT_COORD_SIZE, size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class MPC_ROS_Quadruped : public ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>> Ptr;

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr 					quadruped_interface_ptr_t;
	typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
	typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
	typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

	typedef typename quadruped_interface_t::kinematic_model_t       kinematic_model_t;

	enum {
		rbd_state_dim_ = quadruped_interface_t::rbd_state_dim_
	};

	typedef ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM> BASE;
	typedef typename BASE::system_observation_t 	system_observation_t;
	typedef typename BASE::scalar_t					scalar_t;
	typedef typename BASE::scalar_array_t			scalar_array_t;
	typedef typename BASE::size_array_t				size_array_t;
	typedef typename BASE::state_vector_t			state_vector_t;
	typedef typename BASE::state_vector_array_t		state_vector_array_t;
	typedef typename BASE::input_vector_t			input_vector_t;
	typedef typename BASE::input_vector_array_t		input_vector_array_t;
	typedef typename BASE::input_state_matrix_t		input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t input_state_matrix_array_t;

	typedef typename BASE::mode_sequence_template_t mode_sequence_template_t;

	typedef typename BASE::cost_desired_trajectories_t cost_desired_trajectories_t;

	/**
	 * Constructor
	 *
	 * @param [in] ocs2QuadrupedInterfacePtr: A shared pointer to the quadruped interface class.
	 * @param [in] robotName: The name's of the robot.
	 */
	MPC_ROS_Quadruped(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	/**
	 * Destructor
	 */
	virtual ~MPC_ROS_Quadruped() = default;

	/**
	 * Resets the class to its instantiate state.
	 *
	 * @param [in] initCostDesiredTrajectories: The initial desired cost trajectories.
	 */
	virtual void reset(const cost_desired_trajectories_t& initCostDesiredTrajectories) override;

	/**
	 * This method will be called either after the very fist call of the class or after a call to reset().
	 * Users can use this function for any sort of initialization that they may need in the first call.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 */
	virtual void initCall(
			const system_observation_t& initObservation) override;

	/**
	 * Adjusts the user-defined target trajectories for the cost based on the current observation.
	 *
	 * @param [in] currentObservation: The current observation.
	 * @param costDesiredTrajectories: The received user-defined target trajectories which can be modified based on the current observation.
	 */
	virtual void adjustTargetTrajectories(
			const system_observation_t& currentObservation,
			cost_desired_trajectories_t& costDesiredTrajectories) override;

	/**
	 * Provides the initial mode sequence for time-triggered hybrid systems.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 */
	virtual void initModeSequence(
			const system_observation_t& initObservation) override {}

	/**
	 * Adjusts the user-defined mode sequence for time-triggered hybrid systems based on the current observation.
	 *
	 * @param [in] currentObservation: The current observation.
	 * @param newLogicRulesTemplate: New logicRules template which can be modified based on the current observation.
	 */
	virtual void adjustModeSequence(
			const system_observation_t& currentObservation,
			mode_sequence_template_t& newLogicRulesTemplate) override {}

protected:
	scalar_t estimeTimeToTarget(
			const Eigen::Matrix<scalar_t,2,1>& xyDisplacement) const;

	void targetPoseToDesiredTrajectories(
			const scalar_t& currentTime,
			const state_vector_t& currentState,
			const scalar_t& startDelay,
			const scalar_t& targetReachingDuration,
			const base_coordinate_t& targetPoseDisplacement,
			const base_coordinate_t& targetVelocity,
			cost_desired_trajectories_t& costDesiredTrajectories);

	/*
	 * Variables
	 */
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;

	state_vector_t initState_;
	rbd_state_vector_t defaultConfiguration_;
	input_vector_t initInput_;

};

} // end of namespace switched_model

#include "implementation/MPC_ROS_Quadruped.h"


#endif /* MPC_ROS_QUADRUPED_H_ */
