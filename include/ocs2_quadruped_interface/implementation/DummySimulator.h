/*
 * DummySimulator.h
 *
 *  Created on: Apr 11, 2018
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
DummySimulator<JOINT_COORD_SIZE>::DummySimulator(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const std::string& robotName /*= "robot"*/)

	:  ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
	 , robotName_(robotName)
	 , mrtPtr_( new mrt_t(ocs2QuadrupedInterfacePtr, robotName))
{
#ifdef SAVE_ROS_BAG
	bag_.open("ocs2_hyq_traj.bag", rosbag::bagmode::Write);
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
DummySimulator<JOINT_COORD_SIZE>::~DummySimulator() {

#ifdef SAVE_ROS_BAG
	std::cerr << "ROS Bag file is being saved ... " << std::endl;
	bag_.write("xpp/trajectory_des", startTime_, robotStateCartesianTrajectoryMsg_);
	bag_.close();
	std::cerr << "ROS Bag file is saved. " << std::endl;
#endif
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void DummySimulator<JOINT_COORD_SIZE>::init(int argc, char* argv[], const scalar_t& mcLoopFrequency) {

	mrtPtr_->launchNodes(argc, argv);

	mcLoopFrequency_ = mcLoopFrequency;

	stanceLegs_ = contact_flag_t{1,1,1,1};

	time_ = 0.0;
	ocs2QuadrupedInterfacePtr_->getLoadedInitialState(rbdState_);

	xppVisualizerLaunch(argc, argv);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void DummySimulator<JOINT_COORD_SIZE>::run() {

	::ros::Rate rosRate(mcLoopFrequency_); // in Hz

	vector_3d_array_t o_feetPositionRef;
	vector_3d_array_t o_feetVelocityRef;
	vector_3d_array_t o_feetAccelerationRef;
	vector_3d_array_t o_feetForceRef;
	base_coordinate_t o_comPoseRef;
	base_coordinate_t o_comVelocityRef;
	base_coordinate_t o_comAccelerationRef;

	while(::ros::ok()) {

		mrtPtr_->updateNodes(stanceLegs_, time_, rbdState_, mcLoopFrequency_);

		// waits for the inial MPC plan
		if (mrtPtr_->initialPlanReceived() == false) {
			::ros::Duration(1.0/mcLoopFrequency_).sleep();
			continue;
		}

		time_ += (1.0/mcLoopFrequency_);

		mrtPtr_->computePlan(time_,
				o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef,
				o_comPoseRef, o_comVelocityRef, o_comAccelerationRef,
				stanceLegs_);

		// compute RBD state
		state_vector_t stateRef;
		input_vector_t inputRef;
		mrtPtr_->getComputedReferences(stateRef, inputRef);
		ocs2QuadrupedInterfacePtr_->computeRbdModelState(stateRef, inputRef, rbdState_);

		// contact forces
		for (size_t i=0; i<4; i++)
			o_feetForceRef[i] = inputRef.template segment<3>(3*i);

		xppVisualizerPublish(time_,
				rbdState_.template head<6>(), rbdState_.template segment<6>(18),
				o_feetPositionRef, o_feetVelocityRef, o_feetAccelerationRef, o_feetForceRef);

		rosRate.sleep();

	}  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void DummySimulator<JOINT_COORD_SIZE>::xppVisualizerLaunch(int argc, char* argv[]) {

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
template <size_t JOINT_COORD_SIZE>
void DummySimulator<JOINT_COORD_SIZE>::xppVisualizerPublish(
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

