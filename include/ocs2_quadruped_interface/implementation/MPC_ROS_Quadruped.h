/*
 * MPC_ROS_Quadruped.h
 *
 *  Created on: May 27, 2018
 *      Author: farbod
 */

namespace switched_model {


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::MPC_ROS_Quadruped(
		const quadruped_interface_ptr_t& ocs2QuadrupedInterfacePtr,
		const std::string& robotName /*robot*/)

	: BASE(ocs2QuadrupedInterfacePtr->getMPC(), robotName)
	, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
{
	ocs2QuadrupedInterfacePtr_->getLoadedInitialState(defaultConfiguration_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::reset() {

	BASE::reset();

	initState_.setZero();
	ocs2QuadrupedInterfacePtr_->getLoadedInitialState(defaultConfiguration_);
	initInput_.setZero();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::initCall(
		const system_observation_t& initObservation) {

	// the default state
	initState_ = initObservation.state();

	// Designs weight compensating input.
	ocs2QuadrupedInterfacePtr_->designWeightCompensatingInput(initState_, initInput_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::initGoalState(
		const system_observation_t& initObservation,
		cost_desired_trajectories_t& costDesiredTrajectories) {

	const scalar_t targetReachingDuration = 1.0;
	const base_coordinate_t targetPoseDisplacement = base_coordinate_t::Zero();
	const base_coordinate_t targetVelocity = base_coordinate_t::Zero();

	// costDesiredTrajectories
	targetPoseToDesiredTrajectories(
			initObservation.time(), initObservation.state(),
			0.0,
			targetReachingDuration, targetPoseDisplacement, targetVelocity,
			costDesiredTrajectories);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::adjustTargetTrajectories(
		const system_observation_t& currentObservation,
		cost_desired_trajectories_t& costDesiredTrajectories) {

	if (costDesiredTrajectories.desiredStateTrajectory().size()==1) {
//		// targetPoseDisplacement
//		base_coordinate_t targetPoseDisplacement1, targetVelocity1;
//		ocs2::TargetPoseTransformation<scalar_t>::toTargetPoseDisplacement(costDesiredTrajectories.desiredStateTrajectory()[0],
//				targetPoseDisplacement1, targetVelocity1);

		typename cost_desired_trajectories_t::scalar_array_t& tDesiredTrajectory =
				costDesiredTrajectories.desiredTimeTrajectory();
		typename cost_desired_trajectories_t::dynamic_vector_array_t& xDesiredTrajectory =
				costDesiredTrajectories.desiredStateTrajectory();
		typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
				costDesiredTrajectories.desiredInputTrajectory();


//		// costDesiredTrajectories
//		targetPoseToDesiredTrajectories(
//				currentObservation.time(), currentObservation.state(),
//				ocs2QuadrupedInterfacePtr_->modelSettings().mpcGoalCommandDelay_,
//				targetReachingDuration, targetPoseDisplacement1, targetVelocity1,
//				costDesiredTrajectories);

		scalar_t timeToTarget;
		if (tDesiredTrajectory[0]<0) {
			// x direction
			size_t numReqiredStepsX = std::ceil(
					std::abs(xDesiredTrajectory[0](4)) / (1.0*ocs2QuadrupedInterfacePtr_->strideLength()) );
			// y direction
			size_t numReqiredStepsY = std::ceil(
					std::abs(xDesiredTrajectory[0](5)) / (0.5*ocs2QuadrupedInterfacePtr_->strideLength()) );
			// max
			size_t numReqiredSteps = std::max(numReqiredStepsX, numReqiredStepsY);

			// targetReachingDuration
			timeToTarget = numReqiredSteps * ocs2QuadrupedInterfacePtr_->numPhasesInfullGaitCycle()
										* ocs2QuadrupedInterfacePtr_->strideTime();

		} else {
			timeToTarget = tDesiredTrajectory[0];
		}

		// Desired time trajectory
		tDesiredTrajectory.resize(2);
		tDesiredTrajectory[0] = currentObservation.time() + ocs2QuadrupedInterfacePtr_->modelSettings().mpcGoalCommandDelay_;
		tDesiredTrajectory[1] = currentObservation.time() + timeToTarget + ocs2QuadrupedInterfacePtr_->modelSettings().mpcGoalCommandDelay_;

		// current orientation
		Eigen::Quaternion<scalar_t> q_m =
				Eigen::AngleAxis<scalar_t>(
						currentObservation.state()(0), Eigen::Vector3d::UnitX()) *  // roll
						Eigen::AngleAxis<scalar_t>(
								currentObservation.state()(1), Eigen::Vector3d::UnitY()) *  // pitch
								Eigen::AngleAxis<scalar_t>(
										currentObservation.state()(2), Eigen::Vector3d::UnitZ());   // yaw
		// desired pose changes
		Eigen::Quaternion<scalar_t> q_d(
				xDesiredTrajectory[0](0),
				xDesiredTrajectory[0](1),
				xDesiredTrajectory[0](2),
				xDesiredTrajectory[0](3));

		Eigen::Quaternion<scalar_t> qxyz = q_d*q_m;

		base_coordinate_t targetPose;
		targetPose.template head<3>() = qxyz.toRotationMatrix().eulerAngles(0, 1, 2);
		targetPose.template tail<3>() = qxyz.toRotationMatrix() * (
				xDesiredTrajectory[0].template segment<3>(4) + currentObservation.state().template segment<3>(3));
//		targetPose(3) = xDesiredTrajectory[0](4) + currentObservation.state()(3);  // x
//		targetPose(4) = xDesiredTrajectory[0](5) + currentObservation.state()(4);  // y
//		targetPose(5) = xDesiredTrajectory[0](6) + currentObservation.state()(5);  // z

		base_coordinate_t targetVelocity;
		targetVelocity(0) = xDesiredTrajectory[0](7);
		targetVelocity(1) = xDesiredTrajectory[0](8);
		targetVelocity(2) = xDesiredTrajectory[0](9);
		targetVelocity(3) = xDesiredTrajectory[0](10);
		targetVelocity(4) = xDesiredTrajectory[0](11);
		targetVelocity(5) = xDesiredTrajectory[0](12);


		// Desired state trajectory
		xDesiredTrajectory.resize(2);
		xDesiredTrajectory[0].resize(STATE_DIM);
		xDesiredTrajectory[0].setZero();
		xDesiredTrajectory[0].template head<12>() = currentObservation.state().template head<12>();
		xDesiredTrajectory[0].template segment<12>(12) = defaultConfiguration_.template segment<12>(6);

		xDesiredTrajectory[1].resize(STATE_DIM);
		xDesiredTrajectory[1].setZero();
		// Roll and pitch from initialization
		xDesiredTrajectory[1].template segment<6>(0) = targetPose;
		// target velocities
		xDesiredTrajectory[1].template segment<6>(6) = targetVelocity;
		// joint angle from initialization
		xDesiredTrajectory[1].template segment<12>(12) = defaultConfiguration_.template segment<12>(6);

		// Desired input trajectory
		uDesiredTrajectory.resize(2);
		uDesiredTrajectory[0] = initInput_;
		uDesiredTrajectory[1] = initInput_;

	} else {

		const size_t N = costDesiredTrajectories.desiredStateTrajectory().size();
		costDesiredTrajectories.desiredInputTrajectory().resize(N);
		for (size_t i=0; i<N; i++) {
			// time
			costDesiredTrajectories.desiredTimeTrajectory().at(i) += currentObservation.time();

			// state
			costDesiredTrajectories.desiredStateTrajectory().at(i).conservativeResize(STATE_DIM);
			// base z from initialization
			costDesiredTrajectories.desiredStateTrajectory().at(i).template segment<1>(5) =
					defaultConfiguration_. template segment<1>(5);
			// joint angle from initialization
			costDesiredTrajectories.desiredStateTrajectory().at(i).template segment<12>(12) =
					defaultConfiguration_.template segment<12>(6);
			// input
			costDesiredTrajectories.desiredInputTrajectory().at(i).resize(INPUT_DIM);
			costDesiredTrajectories.desiredInputTrajectory().at(i) = initInput_;
		} // end of i loop
	}
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::targetPoseToDesiredTrajectories(
		const scalar_t& currentTime,
		const state_vector_t& currentState,
		const scalar_t& startDelay,
		const scalar_t& targetReachingDuration,
		const base_coordinate_t& targetPoseDisplacement,
		const base_coordinate_t& targetVelocity,
		cost_desired_trajectories_t& costDesiredTrajectories) {

	// Desired time trajectory
	scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
	tDesiredTrajectory.resize(2);
	tDesiredTrajectory[0] = currentTime + startDelay;
	tDesiredTrajectory[1] = currentTime + startDelay + targetReachingDuration;

	// Desired state trajectory
	typename cost_desired_trajectories_t::dynamic_vector_array_t& xDesiredTrajectory =
			costDesiredTrajectories.desiredStateTrajectory();
	xDesiredTrajectory.resize(2);
	xDesiredTrajectory[0].resize(STATE_DIM);
	xDesiredTrajectory[0].setZero();
	xDesiredTrajectory[0].template head<6>()     = currentState.template head<6>();
	xDesiredTrajectory[0].template segment<6>(6) = currentState.template segment<6>(6);
	xDesiredTrajectory[0].template segment<12>(12) = defaultConfiguration_.template segment<12>(6);

	xDesiredTrajectory[1].resize(STATE_DIM);
	xDesiredTrajectory[1].setZero();
	// Roll and pitch from initialization
	xDesiredTrajectory[1].template segment<2>(0) = defaultConfiguration_. template segment<2>(0)   + targetPoseDisplacement.template segment<2>(0);
	// Yaw from initialization
	xDesiredTrajectory[1].template segment<1>(2) = initState_.template segment<1>(2)   + targetPoseDisplacement.template segment<1>(2);
	// base x, y, and z relative to current state
	xDesiredTrajectory[1].template segment<3>(3) = currentState. template segment<3>(3) + targetPoseDisplacement.template segment<3>(3);
	// target velocities
	xDesiredTrajectory[1].template segment<6>(6) = targetVelocity;
	// joint angle from initialization
	xDesiredTrajectory[1].template segment<12>(12) = defaultConfiguration_.template segment<12>(6);

	// Desired input trajectory
	typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
			costDesiredTrajectories.desiredInputTrajectory();
	uDesiredTrajectory.resize(2);
	uDesiredTrajectory[0] = initInput_;
	uDesiredTrajectory[1] = initInput_;
}


} // end of namespace switched_model
