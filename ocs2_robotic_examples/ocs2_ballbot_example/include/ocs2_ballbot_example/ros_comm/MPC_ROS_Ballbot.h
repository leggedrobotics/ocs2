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

#ifndef MPC_ROS_BALLBOT_OCS2_H_
#define MPC_ROS_BALLBOT_OCS2_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>
#include "ocs2_ballbot_example/definitions.h"

namespace ocs2 {
namespace ballbot {

class MPC_ROS_Ballbot : public ocs2::MPC_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE = ocs2::MPC_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;

	using scalar_t = typename BASE::scalar_t;
	using scalar_array_t = typename BASE::scalar_array_t;
	using size_array_t = typename BASE::size_array_t;
	using state_vector_t = typename BASE::state_vector_t;
	using state_vector_array_t = typename BASE::state_vector_array_t;
	using state_vector_array2_t = typename BASE::state_vector_array2_t;
	using input_vector_t = typename BASE::input_vector_t;
	using input_vector_array_t = typename BASE::input_vector_array_t;
	using input_vector_array2_t = typename BASE::input_vector_array2_t;
	using controller_t = typename BASE::controller_t;
	using input_state_matrix_t = typename BASE::input_state_matrix_t;
	using input_state_matrix_array_t = typename BASE::input_state_matrix_array_t;

	using cost_desired_trajectories_t = ocs2::CostDesiredTrajectories<scalar_t>;

	using target_pose_transformation_t = TargetPoseTransformation<scalar_t>;
	using system_observation_t = ocs2::SystemObservation<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;

	using ros_msg_conversions_t = ocs2::RosMsgConversions<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;

	/**
	 * Default constructor
	 */
	MPC_ROS_Ballbot() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpc: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	MPC_ROS_Ballbot(
			mpc_t &mpc,
			const std::string &robotName = "robot")
	: BASE(mpc, robotName)
	{}

	/**
	 * Destructor.
	 */
	~MPC_ROS_Ballbot() override = default;

	/**
	 * Provides the initial target trajectories for the cost function.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 * @param [out] costDesiredTrajectories: The desired cost trajectories.
	 */
	virtual void initGoalState(
			const system_observation_t &initObservation,
			cost_desired_trajectories_t &costDesiredTrajectories) {

		costDesiredTrajectories.desiredTimeTrajectory().resize(2);
		costDesiredTrajectories.desiredTimeTrajectory().at(0) = 0.0;
		costDesiredTrajectories.desiredTimeTrajectory().at(1) = 1.0;
		costDesiredTrajectories.desiredStateTrajectory().resize(2);
		costDesiredTrajectories.desiredStateTrajectory().at(0) = initObservation.state();
		costDesiredTrajectories.desiredStateTrajectory().at(1) = initObservation.state();
		costDesiredTrajectories.desiredStateTrajectory().at(1).tail<5>().setZero();
		costDesiredTrajectories.desiredInputTrajectory().resize(2);
		costDesiredTrajectories.desiredInputTrajectory().at(0) = input_vector_t::Zero();
		costDesiredTrajectories.desiredInputTrajectory().at(1) = input_vector_t::Zero();
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

		// Received command
		const Eigen::Matrix<scalar_t, 2, 1> targetPosition =
				costDesiredTrajectories.desiredStateTrajectory().front().head<2>();
		double targetYawDisplacement = costDesiredTrajectories.desiredStateTrajectory().front()(2);
		const Eigen::Matrix<scalar_t, 3, 1> targetVelocity =
				costDesiredTrajectories.desiredStateTrajectory().front().tail<3>();

		// Target reaching duration
//		const scalar_t averageSpeed = 2.0;
//		scalar_t targetReachingDuration1 = targetPoseDisplacement.norm() / averageSpeed;
//		const scalar_t averageAcceleration = 10.0;
//		scalar_t targetReachingDuration2 = targetVelocity.norm() / averageAcceleration;
//		scalar_t targetReachingDuration = std::max(targetReachingDuration1, targetReachingDuration2);


		// Desired time trajectory
		scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
		tDesiredTrajectory.resize(2);
		tDesiredTrajectory[0] = currentObservation.time();
//        tDesiredTrajectory[1] = currentObservation.time() + targetReachingDuration;
		tDesiredTrajectory[1] = currentObservation.time() + 0.0025;

		// Desired state trajectory
		typename cost_desired_trajectories_t::dynamic_vector_array_t& xDesiredTrajectory =
				costDesiredTrajectories.desiredStateTrajectory();
		xDesiredTrajectory.resize(2);
		xDesiredTrajectory[0] = currentObservation.state();
		xDesiredTrajectory[1] = currentObservation.state();
//		xDesiredTrajectory[1].head<3>() += targetPoseDisplacement;
        xDesiredTrajectory[1].head<2>() = targetPosition;
        xDesiredTrajectory[1](2) += targetYawDisplacement;
		xDesiredTrajectory[1].tail<5>() << targetVelocity, 0.0, 0.0;

		// Desired input trajectory
		typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
				costDesiredTrajectories.desiredInputTrajectory();
		uDesiredTrajectory.resize(2);
		uDesiredTrajectory[0] = input_vector_t::Zero();
		uDesiredTrajectory[1] = input_vector_t::Zero();
	}

private:

};

} // namespace ballbot
} // namespace ocs2

#endif /* MPC_ROS_BALLBOT_OCS2_H_ */
