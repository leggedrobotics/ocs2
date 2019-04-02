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

#ifndef MPC_ROS_DOUBLE_INTEGRATOR_OCS2_H_
#define MPC_ROS_DOUBLE_INTEGRATOR_OCS2_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include "ocs2_double_integrator_example/definitions.h"

namespace ocs2 {
namespace double_integrator {

class MPC_ROS_Linear_System : public MPC_ROS_Interface<double_integrator_dims::STATE_DIM_, double_integrator_dims::INPUT_DIM_> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef MPC_ROS_Interface<double_integrator_dims::STATE_DIM_, double_integrator_dims::INPUT_DIM_> BASE;

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

	typedef SystemObservation<double_integrator_dims::STATE_DIM_, double_integrator_dims::INPUT_DIM_> system_observation_t;

	typedef RosMsgConversions<double_integrator_dims::STATE_DIM_, double_integrator_dims::INPUT_DIM_> ros_msg_conversions_t;

	/**
	 * Default constructor
	 */
	MPC_ROS_Linear_System() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpc: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	MPC_ROS_Linear_System(
			mpc_t &mpc,
			const std::string &nodeName = "robot_mpc")
	: BASE(mpc, nodeName) {}

	/**
	 * Destructor.
	 */
	virtual ~MPC_ROS_Linear_System() = default;

	/**
	 * Provides the initial target trajectories for the cost function.
	 *
	 * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
	 * @param [out] costDesiredTrajectories: The desired cost trajectories.
	 */
	virtual void initGoalState(
			const system_observation_t &initObservation,
			cost_desired_trajectories_t &costDesiredTrajectories) {

		costDesiredTrajectories.desiredTimeTrajectory().resize(1);
		costDesiredTrajectories.desiredTimeTrajectory().at(0) = 0.0;
		costDesiredTrajectories.desiredStateTrajectory().resize(1);
		costDesiredTrajectories.desiredStateTrajectory().at(0) = state_vector_t::Zero();
		costDesiredTrajectories.desiredInputTrajectory().resize(1);
		costDesiredTrajectories.desiredInputTrajectory().at(0) = input_vector_t::Zero();
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
		const scalar_t targetPoseDisplacement = costDesiredTrajectories.desiredStateTrajectory().front()(0);
		const scalar_t targetVelocity = costDesiredTrajectories.desiredStateTrajectory().front()(1);

		// Desired time trajectory
		scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
		tDesiredTrajectory.resize(1);
		tDesiredTrajectory[0] = currentObservation.time();

		// Desired state trajectory
		typename cost_desired_trajectories_t::dynamic_vector_array_t& xDesiredTrajectory =
				costDesiredTrajectories.desiredStateTrajectory();
		xDesiredTrajectory.resize(1);
		xDesiredTrajectory[0] = currentObservation.state();
		xDesiredTrajectory[0](0) += targetPoseDisplacement;
		xDesiredTrajectory[0](1) = targetVelocity;

		// Desired input trajectory
		typename cost_desired_trajectories_t::dynamic_vector_array_t& uDesiredTrajectory =
				costDesiredTrajectories.desiredInputTrajectory();
		uDesiredTrajectory.resize(1);
		uDesiredTrajectory[0] = input_vector_t::Zero();
	}

private:

};

} // namespace double_integrator
} // namespace ocs2

#endif /* MPC_ROS_DOUBLE_INTEGRATOR_OCS2_H_ */
