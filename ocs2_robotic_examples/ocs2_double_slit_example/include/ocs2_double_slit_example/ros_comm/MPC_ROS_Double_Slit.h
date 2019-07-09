#pragma once

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

class MpcRosDoubleSlit : public MPC_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using BASE =  MPC_ROS_Interface<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;

	using scalar_t = typename BASE::scalar_t ;
	using scalar_array_t = typename BASE::scalar_array_t ;
	using size_array_t = typename BASE::size_array_t ;
	using state_vector_t = typename BASE::state_vector_t ;
	using state_vector_array_t = typename BASE::state_vector_array_t ;
	using state_vector_array2_t = typename BASE::state_vector_array2_t ;
	using input_vector_t = typename BASE::input_vector_t ;
	using input_vector_array_t = typename BASE::input_vector_array_t ;
	using input_vector_array2_t = typename BASE::input_vector_array2_t ;
	using controller_t = typename BASE::controller_t ;
	using input_state_matrix_t = typename BASE::input_state_matrix_t ;
	using input_state_matrix_array_t = typename BASE::input_state_matrix_array_t ;

	using cost_desired_trajectories_t = CostDesiredTrajectories<scalar_t> ;
	using system_observation_t = SystemObservation<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;
	using ros_msg_conversions_t = RosMsgConversions<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> ;

	/**
	 * Default constructor
	 */
	MpcRosDoubleSlit() = default;

	/**
	 * Constructor.
	 *
	 * @param [in] mpc: The MPC object to be interfaced.
	 * @param [in] robotName: The robot's name.
	 */
	explicit MpcRosDoubleSlit(
			mpc_t &mpc,
			const std::string &nodeName = "robot_mpc")
	: BASE(mpc, nodeName) {}

	/**
	 * Destructor.
	 */
	~MpcRosDoubleSlit() override = default;

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
			cost_desired_trajectories_t& costDesiredTrajectories) final {}

private:

};

} // namespace double_slit
} // namespace ocs2
