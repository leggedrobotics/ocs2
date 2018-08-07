/*
 * TargetTrajectories_ROS_Interface.h
 *
 *  Created on: May 27, 2018
 *      Author: farbod
 */

namespace ocs2{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
TargetTrajectories_ROS_Interface<SCALAR_T>::TargetTrajectories_ROS_Interface(
			const std::string& nodeName /*= "robot_mpc"*/)
		: nodeName_(nodeName)
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void TargetTrajectories_ROS_Interface<SCALAR_T>::publishTargetTrajectories(
		const cost_desired_trajectories_t& costDesiredTrajectories) {

	RosMsgConversions<0, 0>::CreateTargetTrajectoriesMsg(costDesiredTrajectories,
			mpcTargetTrajectoriesMsg_);

	mpcTargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void TargetTrajectories_ROS_Interface<SCALAR_T>::launchNodes(int argc, char* argv[]) {

	// reset counters and variables
	reset();

	// display
	ROS_INFO_STREAM("TargetTrajectories node is setting up ...");

	// setup ROS
	::ros::init(argc, argv, nodeName_+"_mpc_target");
	::ros::NodeHandle nodeHandler;

	mpcTargetTrajectoriesPublisher_ = nodeHandler.advertise<ocs2_comm_interfaces::mpc_target_trajectories>(
			nodeName_ + "_mpc_target", 1, true);

	ros::spinOnce();

	// display
	ROS_INFO_STREAM(nodeName_ + " target trajectories command node is ready.");
}

} // namespace ocs2
