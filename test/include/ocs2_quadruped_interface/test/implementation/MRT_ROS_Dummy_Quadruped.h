/*
 * MRT_ROS_Dummy_Quadruped.h
 *
 *  Created on: May 28, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Dummy_Quadruped(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const scalar_t& mrtDesiredFrequency,
		const std::string& robotName /*= "robot"*/,
		const scalar_t& mpcDesiredFrequency /*= -1*/)

	: BASE(typename BASE::mrt_ptr_t(new mrt_t(ocs2QuadrupedInterfacePtr, robotName)), mrtDesiredFrequency, mpcDesiredFrequency)
	, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
	, robotName_(robotName)
{
#ifdef SAVE_ROS_BAG
	rosbagFile_ = "ocs2_" + robotName_ + "_traj.bag";
	bag_.open(rosbagFile_, rosbag::bagmode::Write);
#endif
}

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MRT_ROS_Dummy_Quadruped(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const typename BASE::mrt_ptr_t mrtPtr,
		const scalar_t& mrtLoopFrequency,
		const std::string& robotName /*= "robot"*/)

		: BASE(mrtPtr, mrtLoopFrequency)
		, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
		, robotName_(robotName)
{
#ifdef SAVE_ROS_BAG
	rosbagFile_ = "ocs2_" + robotName_ + "_traj.bag";
	bag_.open(rosbagFile_, rosbag::bagmode::Write);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::~MRT_ROS_Dummy_Quadruped() {

#ifdef SAVE_ROS_BAG
	ROS_INFO_STREAM("ROS Bag file is being saved to \"" + rosbagFile_ + "\" ...");
	bag_.write("xpp/trajectory_des", startTime_, robotStateCartesianTrajectoryMsg_);
	bag_.close();
	ROS_INFO_STREAM("ROS Bag file is saved.");
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::launchVisualizerNode(int argc, char* argv[]) {

	ros::init(argc, argv, robotName_ + "_visualization_node");

	startTime_ = ros::Time::now();

#ifdef SAVE_ROS_BAG
	robotStateCartesianTrajectoryMsg_.header.stamp = startTime_;
#endif

	ros::NodeHandle n;
	visualizationPublisher_ = n.advertise<xpp_msgs::RobotStateCartesian>(xpp_msgs::robot_state_desired, 1);
	ROS_INFO_STREAM("Waiting for visualization subscriber ...");
	while(ros::ok() && visualizationPublisher_.getNumSubscribers() == 0)
		ros::Rate(100).sleep();
	ROS_INFO_STREAM("Visualization subscriber is connected.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishVisualizer(
		const system_observation_t& observation) {

	vector_3d_array_t o_feetPositionRef;
	vector_3d_array_t o_feetVelocityRef;
	vector_3d_array_t o_feetAccelerationRef;
	vector_3d_array_t o_feetForceRef;
	base_coordinate_t o_comPoseRef;
	base_coordinate_t o_comVelocityRef;
	base_coordinate_t o_comAccelerationRef;
	contact_flag_t 	   stanceLegs;
	rbd_state_vector_t rbdState;

	std::static_pointer_cast<mrt_t>(BASE::mrtPtr_)->computePlan(
			observation.time(),
			o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef,
			o_comPoseRef, o_comVelocityRef, o_comAccelerationRef,
			stanceLegs);

	// compute RBD state
	ocs2QuadrupedInterfacePtr_->computeRbdModelState(observation.state(), observation.input(), rbdState);

	// contact forces
	for (size_t i=0; i<4; i++)
		o_feetForceRef[i] = observation.input().template segment<3>(3*i);

	publishXppVisualizer(observation.time(),
			rbdState.template head<6>(), rbdState.template segment<6>(18),
			o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef, o_feetForceRef);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MRT_ROS_Dummy_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::publishXppVisualizer(
		const scalar_t& time,
		const base_coordinate_t& basePose,
		const base_coordinate_t& baseLocalVelocities,
		const vector_3d_array_t& feetPosition,
		const vector_3d_array_t& feetVelocity,
		const vector_3d_array_t& feetAcceleration,
		const vector_3d_array_t& feetForce) {

	const scalar_t minTimeDifference = 10e-3;

	static scalar_t lastTime = 0.0;
	if (time-lastTime < minTimeDifference)  return;

	lastTime = time;

	//construct the message
	xpp_msgs::RobotStateCartesian point;

	Eigen::Quaternion<scalar_t> qx( cos(basePose(0)/2),   sin(basePose(0)/2),   0.0,   0.0 );
	Eigen::Quaternion<scalar_t> qy( cos(basePose(1)/2),   0.0,   sin(basePose(1)/2),   0.0 );
	Eigen::Quaternion<scalar_t> qz( cos(basePose(2)/2),   0.0,   0.0,   sin(basePose(2)/2) );
	Eigen::Quaternion<scalar_t> qxyz = qz*qy*qx;
	point.base.pose.orientation.x = qxyz.x();
	point.base.pose.orientation.y = qxyz.y();
	point.base.pose.orientation.z = qxyz.z();
	point.base.pose.orientation.w = qxyz.w();
	point.base.pose.position.x = basePose(3);
	point.base.pose.position.y = basePose(4);
	point.base.pose.position.z = basePose(5);

	point.base.twist.linear.x  = baseLocalVelocities(0);
	point.base.twist.linear.y  = baseLocalVelocities(1);
	point.base.twist.linear.z  = baseLocalVelocities(2);
	point.base.twist.angular.x = baseLocalVelocities(3);
	point.base.twist.angular.y = baseLocalVelocities(4);
	point.base.twist.angular.z = baseLocalVelocities(5);

//	point.base.accel =

	point.time_from_start = ros::Duration(time);

	constexpr int numEE = 4;
	point.ee_motion.resize(numEE);
	point.ee_forces.resize(numEE);
	point.ee_contact.resize(numEE);
	for(size_t ee_k=0; ee_k < numEE; ee_k++){
		point.ee_motion[ee_k].pos.x = feetPosition[ee_k](0);
		point.ee_motion[ee_k].pos.y = feetPosition[ee_k](1);
		point.ee_motion[ee_k].pos.z = feetPosition[ee_k](2);

		point.ee_motion[ee_k].vel.x = feetVelocity[ee_k](0);
		point.ee_motion[ee_k].vel.y = feetVelocity[ee_k](1);
		point.ee_motion[ee_k].vel.z = feetVelocity[ee_k](2);

		point.ee_motion[ee_k].acc.x = feetAcceleration[ee_k](0);
		point.ee_motion[ee_k].acc.y = feetAcceleration[ee_k](1);
		point.ee_motion[ee_k].acc.z = feetAcceleration[ee_k](2);

		point.ee_forces[ee_k].x = feetForce[ee_k](0);
		point.ee_forces[ee_k].y = feetForce[ee_k](1);
		point.ee_forces[ee_k].z = feetForce[ee_k](2);

//		point.ee_contact[ee_k] = stanceLegSequene[i][ee_k];
	}

	visualizationPublisher_.publish(point);

#ifdef SAVE_ROS_BAG
	const auto stamp = ros::Time(startTime_.toSec() + time);
	bag_.write("xpp/state_des",stamp, point);
	robotStateCartesianTrajectoryMsg_.points.push_back(point);
#endif

}

}  // end of namespace switched_model

