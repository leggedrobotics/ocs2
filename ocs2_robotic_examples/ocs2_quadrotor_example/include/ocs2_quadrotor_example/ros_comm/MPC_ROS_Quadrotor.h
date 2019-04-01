/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef MPC_ROS_QUADROTOR_OCS2_H_
#define MPC_ROS_QUADROTOR_OCS2_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>
#include "ocs2_quadrotor_example/definitions.h"

namespace ocs2 {
namespace quadrotor {

class MPC_ROS_Quadrotor : public MPC_ROS_Interface<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MPC_ROS_Interface<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> BASE;

	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::scalar_array_t scalar_array_t;
	typedef typename BASE::size_array_t size_array_t;
	typedef typename BASE::state_vector_t state_vector_t;
	typedef typename BASE::state_vector_array_t state_vector_array_t;
	typedef typename BASE::state_vector_array2_t state_vector_array2_t;
	typedef typename BASE::input_vector_t input_vector_t;
	typedef typename BASE::input_vector_array_t input_vector_array_t;
	typedef typename BASE::input_vector_array2_t input_vector_array2_t;
	typedef typename BASE::controller_t controller_t;
	typedef typename BASE::controller_array_t controller_array_t;
	typedef typename BASE::input_state_matrix_t input_state_matrix_t;
	typedef typename BASE::input_state_matrix_array_t input_state_matrix_array_t;

	typedef CostDesiredTrajectories<scalar_t> cost_desired_trajectories_t;

	typedef TargetPoseTransformation<scalar_t> target_pose_transformation_t;
	typedef target_pose_transformation_t::pose_vector_t pose_vector_t;

	typedef SystemObservation<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> system_observation_t;

	typedef RosMsgConversions<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> ros_msg_conversions_t;

	/**
	 * Default constructor
	 */
	MPC_ROS_Quadrotor() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpc: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	MPC_ROS_Quadrotor(
			mpc_t &mpc,
			const std::string &robotName = "robot")
	: BASE(mpc, robotName) {}

	/**
	 * Destructor.
	 */
	virtual ~MPC_ROS_Quadrotor() = default;

	/**
	 * Provides the initial target trajectories for the cost function.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 * @param [out] costDesiredTrajectories: The desired cost trajectories.
	 */
	void initGoalState(
			const system_observation_t &initObservation,
			cost_desired_trajectories_t &costDesiredTrajectories) final {

		const scalar_t targetReachingDuration = 1.0;
		const pose_vector_t targetPoseDisplacement = pose_vector_t::Zero();
		const pose_vector_t targetVelocity = pose_vector_t::Zero();

		// costDesiredTrajectories
		targetPoseToDesiredTrajectories(
				initObservation.time(), initObservation.state(),
				0.0 /*startDelay*/,
				targetReachingDuration, targetPoseDisplacement, targetVelocity,
				costDesiredTrajectories);
	}

	/**
	 * Adjusts the user-defined target trajectories for the cost based on the current observation.
	 *
	 * @param [in] currentObservation: The current observation.
	 * @param costDesiredTrajectories: The received user-defined target trajectories which can be modified
	 * based on the current observation.
	 */
	void adjustTargetTrajectories(
			const system_observation_t& currentObservation,
			cost_desired_trajectories_t& costDesiredTrajectories) final {

		// targetPoseDisplacement
		pose_vector_t targetPoseDisplacement, targetVelocity;
		TargetPoseTransformation<scalar_t>::toTargetPoseDisplacement(costDesiredTrajectories.desiredStateTrajectory()[0],
				targetPoseDisplacement, targetVelocity);

		// reversing the order of the position and orientation.
		Eigen::Matrix<scalar_t, 3, 1> temp;
		temp = targetPoseDisplacement.head<3>();
		targetPoseDisplacement.head<3>() = targetPoseDisplacement.tail<3>();
		targetPoseDisplacement.tail<3>() = temp;
		temp = targetVelocity.head<3>();
		targetVelocity.head<3>() = targetVelocity.tail<3>();
		targetVelocity.tail<3>() = temp;

		// targetReachingDuration
		const scalar_t averageSpeed = 2.0;
		scalar_t targetReachingDuration1 = targetPoseDisplacement.norm() / averageSpeed;
		const scalar_t averageAcceleration = 10.0;
		scalar_t targetReachingDuration2 = targetVelocity.norm() / averageAcceleration;
		scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);

		// costDesiredTrajectories
		targetPoseToDesiredTrajectories(
				currentObservation.time(), currentObservation.state(),
				0.0 /*startDelay*/,
				targetReachingDuration, targetPoseDisplacement, targetVelocity,
				costDesiredTrajectories);
	}

private:
	void targetPoseToDesiredTrajectories(
			const scalar_t& currentTime,
			const state_vector_t& currentState,
			const scalar_t& startDelay,
			const scalar_t& targetReachingDuration,
			const pose_vector_t& targetPoseDisplacement,
			const pose_vector_t& targetVelocity,
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
		xDesiredTrajectory[0].resize(quadrotor::STATE_DIM_);
		xDesiredTrajectory[0].setZero();
		xDesiredTrajectory[0].template segment<6>(0) = currentState.template segment<6>(0);
		xDesiredTrajectory[0].template segment<6>(6) = currentState.template segment<6>(6);

		xDesiredTrajectory[1].resize(quadrotor::STATE_DIM_);
		xDesiredTrajectory[1].setZero();
		xDesiredTrajectory[1].template segment<6>(0) = currentState. template segment<6>(0) + targetPoseDisplacement;
		xDesiredTrajectory[1].template segment<6>(6) = targetVelocity;

		// Desired input trajectory
		typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
				costDesiredTrajectories.desiredInputTrajectory();
		uDesiredTrajectory.resize(2);
		uDesiredTrajectory[0] = input_vector_t::Zero();
		uDesiredTrajectory[1] = input_vector_t::Zero();
	}
};

} // namespace quadrotor
} // namespace ocs2

#endif /* MPC_ROS_QUADROTOR_OCS2_H_ */
