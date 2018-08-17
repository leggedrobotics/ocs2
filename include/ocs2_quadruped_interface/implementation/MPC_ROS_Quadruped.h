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
{}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, size_t STATE_DIM, size_t INPUT_DIM>
void MPC_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>::reset() {

	BASE::reset();

	initState_.setZero();
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

	// targetPoseDisplacement
	base_coordinate_t targetPoseDisplacement, targetVelocity;
	ocs2::TargetPoseTransformation<scalar_t>::toTargetPoseDisplacement(costDesiredTrajectories.desiredStateTrajectory()[0],
			targetPoseDisplacement, targetVelocity);

	// x direction
	size_t numReqiredStepsX = std::ceil(
			std::abs(targetPoseDisplacement(3)) / (1.0*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// y direction
	size_t numReqiredStepsY = std::ceil(
			std::abs(targetPoseDisplacement(4)) / (0.5*ocs2QuadrupedInterfacePtr_->strideLength()) );
	// max
	size_t numReqiredSteps = std::max(numReqiredStepsX, numReqiredStepsY);

	// targetReachingDuration
	scalar_t targetReachingDuration = numReqiredSteps * ocs2QuadrupedInterfacePtr_->numPhasesInfullGaitCycle()
			* ocs2QuadrupedInterfacePtr_->strideTime();

	// costDesiredTrajectories
	targetPoseToDesiredTrajectories(
			currentObservation.time(), currentObservation.state(),
			ocs2QuadrupedInterfacePtr_->modelSettings().mpcGoalCommandDelay_,
			targetReachingDuration, targetPoseDisplacement, targetVelocity,
			costDesiredTrajectories);
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
	xDesiredTrajectory[0].template segment<12>(12) = initState_.template segment<12>(12);

	xDesiredTrajectory[1].resize(STATE_DIM);
	xDesiredTrajectory[1].setZero();
	xDesiredTrajectory[1].template segment<3>(0) = initState_. template segment<3>(0)   + targetPoseDisplacement.template segment<3>(0);
	xDesiredTrajectory[1].template segment<2>(3) = currentState. template segment<2>(3) + targetPoseDisplacement.template segment<2>(3);
	xDesiredTrajectory[1].template segment<1>(5) = initState_. template segment<1>(5)   + targetPoseDisplacement.template segment<1>(5);
	xDesiredTrajectory[1].template segment<6>(6) = targetVelocity;
	xDesiredTrajectory[1].template segment<12>(12) = initState_.template segment<12>(12);

	// Desired input trajectory
	typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
			costDesiredTrajectories.desiredInputTrajectory();
	uDesiredTrajectory.resize(2);
	uDesiredTrajectory[0] = initInput_;
	uDesiredTrajectory[1] = initInput_;
}


} // end of namespace switched_model
