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

	: BASE(ocs2QuadrupedInterfacePtr->getMPCPtr().get(), robotName)
	, ocs2QuadrupedInterfacePtr_(ocs2QuadrupedInterfacePtr)
{
	ocs2QuadrupedInterfacePtr_->getLoadedInitialState(defaultConfiguration_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::reset(
		const cost_desired_trajectories_t& initCostDesiredTrajectories) {

	BASE::reset(initCostDesiredTrajectories);

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
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::adjustTargetTrajectories(
		const system_observation_t& currentObservation,
		cost_desired_trajectories_t& costDesiredTrajectories) {

	if (costDesiredTrajectories.desiredStateTrajectory().size()==1) {

		// time to reach target
		auto& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
		scalar_t timeToTarget;
		if (tDesiredTrajectory[0] < 0) {
			timeToTarget = estimeTimeToTarget(
					costDesiredTrajectories.desiredStateTrajectory()[0].template segment<2>(4));
		} else {
			timeToTarget = tDesiredTrajectory[0];
		}

		// targetPoseDisplacement
		base_coordinate_t targetPoseDisplacement, targetVelocity;
		ocs2::TargetPoseTransformation<scalar_t>::toTargetPoseDisplacement(
				costDesiredTrajectories.desiredStateTrajectory()[0],
				targetPoseDisplacement, targetVelocity);

		// costDesiredTrajectories
		targetPoseToDesiredTrajectories(
				currentObservation.time(), currentObservation.state(),
				ocs2QuadrupedInterfacePtr_->modelSettings().mpcGoalCommandDelay_,
				timeToTarget, targetPoseDisplacement, targetVelocity,
				costDesiredTrajectories);
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
typename MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::scalar_t
MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::estimeTimeToTarget(
		const Eigen::Matrix<scalar_t,2,1>& xyDisplacement) const {

	// x direction
	size_t numReqiredStepsX = std::ceil(
			std::abs(xyDisplacement(0)) / (1.0*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// y direction
	size_t numReqiredStepsY = std::ceil(
			std::abs(xyDisplacement(1)) / (0.5*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// max
	size_t numReqiredSteps = std::max(numReqiredStepsX, numReqiredStepsY);

	// targetReachingDuration
	scalar_t timeToTarget = numReqiredSteps * ocs2QuadrupedInterfacePtr_->numPhasesInfullGaitCycle()
			* ocs2QuadrupedInterfacePtr_->strideTime();

	return timeToTarget;
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
	// base x, y relative to current state
	xDesiredTrajectory[1].template segment<2>(3) = currentState. template segment<2>(3) + targetPoseDisplacement.template segment<2>(3);
	// base z from initialization
	xDesiredTrajectory[1].template segment<1>(5) = defaultConfiguration_. template segment<1>(5) + targetPoseDisplacement.template segment<1>(5);
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
