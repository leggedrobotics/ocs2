/*
 * TargetTrajectories_ROS_Quadruped.h
 *
 *  Created on: May 27, 2018
 *      Author: farbod
 */

#ifndef TARGETTRAJECTORIES_ROS_QUADRUPED_H_
#define TARGETTRAJECTORIES_ROS_QUADRUPED_H_

#include <iomanip>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/command/TargetTrajectories_ROS_Interface.h>

#include "ocs2_quadruped_interface/TargetPoseCommand.h"

namespace switched_model {

/**
 * This class implements TargetTrajectories communication using ROS.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class TargetTrajectories_ROS_Quadruped : public ocs2::TargetTrajectories_ROS_Interface<SCALAR_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::TargetTrajectories_ROS_Interface<SCALAR_T> BASE;
	typedef typename BASE::cost_desired_trajectories_t 	cost_desired_trajectories_t;
	typedef typename BASE::scalar_array_t 				scalar_array_t;
	typedef typename BASE::dynamic_vector_t 			dynamic_vector_t;
	typedef typename BASE::dynamic_vector_array_t 		dynamic_vector_array_t;

	TargetTrajectories_ROS_Quadruped(
				const std::string& robotName = "robot",
				const scalar_array_t& goalPoseLimit = scalar_array_t{45.0, 45.0, 360.0, 2.0, 1.0, 0.3})
	: BASE(robotName)
	, targetPoseCommand_(goalPoseLimit)
	{}

	virtual ~TargetTrajectories_ROS_Quadruped() = default;

	void run() {

		typedef typename TargetPoseCommand<SCALAR_T>::DIM DIM;

		while (ros::ok()) {

			// get command line
			std::cout << "Enter XYZ displacement for the robot, separated by spaces: ";
			typename TargetPoseCommand<SCALAR_T>::pose_vector_t targetPoseDisplacement;
			targetPoseCommand_.getCommandLineTargetPoseDisplacement(targetPoseDisplacement);

			// To cost DesiredState
			dynamic_vector_t desiredState;
			typename TargetPoseCommand<SCALAR_T>::pose_vector_t targetPoseDisplacementLimited =
					targetPoseCommand_.toCostDesiredState(targetPoseDisplacement, desiredState);

			std::cout << "The following position displacement is published: [";
			for (size_t i=0; i<DIM::POSE_DIM_; i++)
				std::cout << std::setprecision(4) << targetPoseDisplacementLimited(i) << ", ";
			std::cout << "\b\b]" << std::endl << std::endl;

			// create the message
			cost_desired_trajectories_t costDesiredTrajectories;
			costDesiredTrajectories.desiredTimeTrajectory().resize(1);
			costDesiredTrajectories.desiredTimeTrajectory()[0] = 0.0;

			costDesiredTrajectories.desiredStateTrajectory().resize(1);
			costDesiredTrajectories.desiredStateTrajectory()[0] = desiredState;

			costDesiredTrajectories.desiredInputTrajectory().resize(1);
			costDesiredTrajectories.desiredInputTrajectory()[0] = dynamic_vector_t::Zero(0);

			BASE::publishTargetTrajectories(costDesiredTrajectories);

		}  // end of while loop
	}

private:
	TargetPoseCommand<SCALAR_T> targetPoseCommand_;

};

} // end of namespace switched_model

#endif /* TARGETTRAJECTORIES_ROS_QUADRUPED_H_ */
