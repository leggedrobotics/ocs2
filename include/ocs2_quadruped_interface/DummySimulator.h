/*
 * DummySimulator.h
 *
 *  Created on: Apr 10, 2018
 *      Author: farbod
 */

#ifndef DUMMYSIMULATOR_H_
#define DUMMYSIMULATOR_H_

#include <unistd.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <xpp_msgs/RobotStateCartesianTrajectory.h>
#include <xpp_msgs/RobotStateCartesian.h>
#include <xpp_msgs/topic_names.h>

#include "ocs2_quadruped_interface/OCS2QuadrupedMRT.h"

#define SAVE_ROS_BAG

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class DummySimulator
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef OCS2QuadrupedInterface<JOINT_COORD_SIZE> quadruped_interface_t;
	typedef typename quadruped_interface_t::Ptr quadruped_interface_ptr_t;

	typedef OCS2QuadrupedMRT<JOINT_COORD_SIZE> 	mrt_t;
	typedef typename mrt_t::Ptr 				mrt_ptr_t;

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

	typedef Eigen::Matrix<scalar_t,3,1>	vector_3d_t;
	typedef std::array<vector_3d_t,4>	vector_3d_array_t;

	/**
	 *
	 * @param ocs2QuadrupedInterfacePtr
	 * @param robotName
	 */
	DummySimulator(
			const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
			const std::string& robotName = "robot");

	/**
	 *
	 */
	~DummySimulator();

	/**
	 * Initializes the MRT node and visualization node.
	 *
	 * @param [in] argc: command line number of inputs.
	 * @param [in] argv: command line inputs' value.
	 * @param [in] mcLoopFrequency: Motion controller frequency
	 */
	void init(int argc, char* argv[], const scalar_t& mcLoopFrequency);

	/**
	 * Runs the dummy simulators. For visualization launch the xpp visualization node
	 */
	void run();

protected:
	/**
	 * Launches the xpp visualization node.
	 * @param [in] argc: command line number of inputs.
	 * @param [in] argv: command line inputs' value.
	 */
	void xppVisualizerLaunch(int argc, char* argv[]);

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
	void xppVisualizerPublish(
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

	mrt_ptr_t mrtPtr_;

	scalar_t mcLoopFrequency_;

	contact_flag_t stanceLegs_;
	scalar_t time_;
	rbd_state_vector_t rbdState_;

	ros::Publisher visualizationPublisher_;
	ros::Time startTime_;

#ifdef SAVE_ROS_BAG
	rosbag::Bag bag_;
	xpp_msgs::RobotStateCartesianTrajectory robotStateCartesianTrajectoryMsg_;
#endif
};

} // end of namespace switched_model

#include "implementation/DummySimulator.h"

#endif /* DUMMYSIMULATOR_H_ */
