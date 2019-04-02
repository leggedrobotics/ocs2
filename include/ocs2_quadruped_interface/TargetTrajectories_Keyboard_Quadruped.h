/*
 * TargetTrajectories_Keyboard_Quadruped.h
 *
 *  Created on: Aug 15, 2018
 *      Author: farbod
 */

#ifndef TARGETTRAJECTORIES_KEYBOARD_QUADRUPED_H_
#define TARGETTRAJECTORIES_KEYBOARD_QUADRUPED_H_

#include <iomanip>

#include <ocs2_robotic_tools/command/TargetTrajectories_Keyboard_Interface.h>
#include <ocs2_robotic_tools/command/TargetPoseTransformation.h>

namespace switched_model {

/**
 * This class implements TargetTrajectories communication using ROS.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class TargetTrajectories_Keyboard_Quadruped : public ocs2::TargetTrajectories_Keyboard_Interface<SCALAR_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		command_dim_ = 12
	};

	enum class COMMAND_MODE
	{
			POSITION,
			VELOCITY
	};

	typedef ocs2::TargetTrajectories_Keyboard_Interface<SCALAR_T> BASE;
	typedef typename BASE::scalar_t scalar_t;
	typedef typename BASE::scalar_array_t scalar_array_t;
	typedef typename BASE::dynamic_vector_t dynamic_vector_t;
	typedef typename BASE::dynamic_vector_array_t dynamic_vector_array_t;
	typedef typename BASE::cost_desired_trajectories_t cost_desired_trajectories_t;

	/**
	 * Constructor.
	 *
	 * @param [in] robotName: The robot's name.
	 * @param [in] goalPoseLimit: Limits for the input command. It has size 12 with following entries.
	 * @param [in] command_mode: Whether to use position mode or velocity mode.
	 *
	 * goalPoseLimit(0): X
	 * goalPoseLimit(1): Y
	 * goalPoseLimit(2): Z
	 *
	 * goalPoseLimit(3): Roll
	 * goalPoseLimit(4): Pitch
	 * goalPoseLimit(5): Yaw
	 *
	 * goalPoseLimit(6): v_X
	 * goalPoseLimit(7): v_Y
	 * goalPoseLimit(8): v_Z
	 *
	 * goalPoseLimit(9): \omega_X
	 * goalPoseLimit(10): \omega_Y
	 * goalPoseLimit(11): \omega_Z
	 */
	TargetTrajectories_Keyboard_Quadruped(
				const std::string& robotName = "robot",
				const scalar_array_t& goalPoseLimit =
						scalar_array_t{2.0, 1.0, 0.3, 45.0, 45.0, 360.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
				const COMMAND_MODE command_mode = COMMAND_MODE::POSITION)
	: BASE(robotName, command_dim_, goalPoseLimit), command_mode_(command_mode)
	{}

	/**
	* Default destructor
	*/
	~TargetTrajectories_Keyboard_Quadruped() = default;

	/**
	 * From command line loaded command to desired time, state, and input.
	 *
	 * @param [out] commadLineTarget: The loaded command target.
	 * @param [in] desiredTime: Desired time to be published.
	 * @param [in] desiredState: Desired state to be published.
	 * @param [in] desiredInput: Desired input to be published.
	 */
	void toCostDesiredTimeStateInput(
			const scalar_array_t& commadLineTarget,
			scalar_t& desiredTime,
			dynamic_vector_t& desiredState,
			dynamic_vector_t& desiredInput) final {

		scalar_array_t commadLineTargetOrdeCorrected(command_dim_);
		if (command_mode_ == COMMAND_MODE::POSITION) {
			// reversing the order of the position and orientation.
			for (size_t j = 0; j < 3; j++) {
				// pose
				commadLineTargetOrdeCorrected[j] = commadLineTarget[3 + j];
				commadLineTargetOrdeCorrected[3 + j] = commadLineTarget[j];
				// velocities
				commadLineTargetOrdeCorrected[6 + j] = commadLineTarget[9 + j];
				commadLineTargetOrdeCorrected[9 + j] = commadLineTarget[6 + j];
			}
		} else if (command_mode_ == COMMAND_MODE::VELOCITY) {
			// reversing the order of the position and orientation.
			// velocity before position and orientation
			for (size_t j = 0; j < 3; j++) {
				// velocities
				commadLineTargetOrdeCorrected[6 + j] = commadLineTarget[3 + j];
				commadLineTargetOrdeCorrected[9 + j] = commadLineTarget[j];
				// pose
				commadLineTargetOrdeCorrected[j] = commadLineTarget[9 + j];
				commadLineTargetOrdeCorrected[3 + j] = commadLineTarget[6 + j];
			}
		} else {
			std::runtime_error("Unknown command mode for target!");
		}

		// time
		desiredTime = -1.0;
		// state
		ocs2::TargetPoseTransformation<scalar_t>::toCostDesiredState(
				commadLineTargetOrdeCorrected, desiredState);
		// input
		desiredInput = dynamic_vector_t::Zero(0);
	}

private:
		COMMAND_MODE command_mode_;
};

} // end of namespace switched_model

#endif /* TARGETTRAJECTORIES_KEYBOARD_QUADRUPED_H_ */
