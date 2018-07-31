/*
 * MRT_ROS_Dummy_Quadruped.h
 *
 *  Created on: May 28, 2018
 *      Author: farbod
 */

#ifndef MRT_ROS_DUMMY_QUADRUPED_H_
#define MRT_ROS_DUMMY_QUADRUPED_H_

#include <unistd.h>

#include <rosbag/bag.h>

#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>
#include "ocs2_quadruped_interface/MRT_ROS_Quadruped.h"

#define SAVE_ROS_BAG

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM=12+JOINT_COORD_SIZE, size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class MRT_ROS_Dummy_Quadruped : public ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, typename OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::logic_rules_t>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr quadruped_interface_ptr_t;

	typedef typename quadruped_interface_t::contact_flag_t			contact_flag_t;
	typedef typename quadruped_interface_t::generalized_coordinate_t generalized_coordinate_t;
	typedef typename quadruped_interface_t::joint_coordinate_t 		joint_coordinate_t;
	typedef typename quadruped_interface_t::base_coordinate_t 		base_coordinate_t;
	typedef typename quadruped_interface_t::rbd_state_vector_t		rbd_state_vector_t;

	enum {
		RBD_STATE_DIM = quadruped_interface_t::RBD_STATE_DIM
	};

	typedef ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM, typename quadruped_interface_t::logic_rules_t> BASE;
	typedef typename BASE::controller_t					controller_t;
	typedef typename BASE::controller_array_t			controller_array_t;
	typedef typename BASE::scalar_t						scalar_t;
	typedef typename BASE::scalar_array_t				scalar_array_t;
	typedef typename BASE::size_array_t					size_array_t;
	typedef typename BASE::state_vector_t 				state_vector_t;
	typedef typename BASE::state_vector_array_t			state_vector_array_t;
	typedef typename BASE::input_vector_t 		 		input_vector_t;
	typedef typename BASE::input_vector_array_t  		input_vector_array_t;
	typedef typename BASE::input_state_matrix_t 	   	input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t  	input_state_matrix_array_t;
	typedef typename BASE::system_observation_t 		system_observation_t;

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t,4>	vector_3d_array_t;

	typedef MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM> mrt_t;
	typedef typename mrt_t::Ptr 				mrt_ptr_t;


	/**
	 * Constructor.
	 *
	 * @param [in] ocs2QuadrupedInterfacePtr
	 * @param [in] mrtDesiredFrequency: MRT loop frequency in Hz
	 * @param [in] robotName
	 * @param [in] mpcDesiredFrequency: MPC loop frequency in Hz
	 */
	MRT_ROS_Dummy_Quadruped(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const scalar_t& mrtDesiredFrequency,
			const std::string& robotName = "robot",
			const scalar_t& mpcDesiredFrequency = -1);

		MRT_ROS_Dummy_Quadruped(
				const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
				const typename BASE::mrt_ptr_t mrtPtr,
				const scalar_t& mrtDesiredFrequency,
				const std::string& robotName = "robot",
				const scalar_t& mpcDesiredFrequency = -1);

	/**
	 * Destructor.
	 */
	virtual ~MRT_ROS_Dummy_Quadruped();


protected:
	/**
	 * A user-defined function which modifies the observation before publishing.
	 *
	 * @param [in] observation: The current observation.
	 */
	virtual void modifyObservation(system_observation_t& observation) override {}

	/**
	 * Launches the visualization node
	 *
	 * @param [in] argc: command line number of inputs.
	 * @param [in] argv: command line inputs' value.
	 */
	virtual void launchVisualizerNode(int argc, char* argv[]) override;

	/**
	 * Visulizes the current observation.
	 *
	 * @param [in] observation: The current observation.
	 */
	virtual void publishVisualizer(const system_observation_t& observation) override;

	/**
	 * Publishes the xpp visualization messages and also if "SAVE_ROS_BAG" is defined, it saves then in a ROS bag file.
	 *
	 * @param time: time.
	 * @param basePose: Base pose in the origin frame.
	 * @param baseLocalVelocities: Base local velocities.
	 * @param feetPosition: Feet position in the origin frame.
	 * @param feetVelocity: Feet velocity in the origin frame.
	 * @param feetAcceleration: Feet acceleration in the origin frame.
	 * @param feetForce: Contact forces acting on the feet in the origin frame.
	 */
	void publishXppVisualizer(
			const scalar_t& time,
			const base_coordinate_t& basePose,
			const base_coordinate_t& baseLocalVelocities,
			const vector_3d_array_t& feetPosition,
			const vector_3d_array_t& feetVelocity,
			const vector_3d_array_t& feetAcceleration,
			const vector_3d_array_t& feetForce);

private:
	quadruped_interface_ptr_t ocs2QuadrupedInterfacePtr_;
	std::string robotName_;
	std::string rosbagFile_;

	ros::Publisher visualizationPublisher_;
	ros::Time startTime_;

#ifdef SAVE_ROS_BAG
	rosbag::Bag bag_;
	xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg_;
#endif
};

} // end of namespace switched_model

#include "implementation/MRT_ROS_Dummy_Quadruped.h"

#endif /* MRT_ROS_DUMMY_QUADRUPED_H_ */
