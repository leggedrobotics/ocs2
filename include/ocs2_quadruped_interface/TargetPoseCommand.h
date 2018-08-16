/*
 * TargetPoseCommand.h
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#ifndef TARGETPOSECOMMAND_H_
#define TARGETPOSECOMMAND_H_

#include <vector>
#include <csignal>
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace switched_model {

/**
 * Target pose
 * targetPoseDisplacementVelocity(0): X
 * targetPoseDisplacementVelocity(1): Y
 * targetPoseDisplacementVelocity(2): Z
 *
 * targetPoseDisplacementVelocity(3): Roll
 * targetPoseDisplacementVelocity(4): Pitch
 * targetPoseDisplacementVelocity(5): Yaw
 *
 * targetPoseDisplacementVelocity(6): v_X
 * targetPoseDisplacementVelocity(7): v_Y
 * targetPoseDisplacementVelocity(8): v_Z
 *
 * targetPoseDisplacementVelocity(9): \omega_X
 * targetPoseDisplacementVelocity(10): \omega_Y
 * targetPoseDisplacementVelocity(11): \omega_Z
 *
 *
 * Desired state:
 * dState = costDesiredTrajectories.desiredStateTrajectory().at(0)
 *
 * dState(0): orientation.w
 * dState(1): orientation.x
 * dState(2): orientation.y
 * dState(3): orientation.z
 *
 * dState(4): position.x
 * dState(5): position.y
 * dState(6): position.z
 *
 * dState(7): angular_velocity.x
 * dState(8): angular_velocity.y
 * dState(9): angular_velocity.y
 *
 * dState(10): linear_velocity.x
 * dState(11): linear_velocity.y
 * dState(12): linear_velocity.z
 *
 */

template <typename SCALAR_T>
class TargetPoseCommand
{
public:
	enum DIM
	{
		POSE_DIM_ = 6,
	};

	typedef SCALAR_T scalar_t;
	typedef std::vector<scalar_t> scalar_array_t;
	typedef Eigen::Matrix<scalar_t, POSE_DIM_, 1> pose_vector_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1> dynamic_vector_t;

	/**
	 * Constructor
	 */
	TargetPoseCommand() = default;

	/**
	 * Destructor
	 */
	~TargetPoseCommand() = default;

	/**
	 * Transforms the cost function's desired state to the target pose displacement.
	 *
	 * @param [in] desiredState: The cost function's desired state.
	 * @param [out] targetPoseDisplacement: The target pose displacement.
	 * @param [out] targetVelocity: The target velocity.
	 */
	static void toTargetPoseDisplacement(
			const dynamic_vector_t& desiredState,
			pose_vector_t& targetPoseDisplacement,
			pose_vector_t& targetVelocity) {

		Eigen::Quaternion<scalar_t> qxyz(
				desiredState(0),
				desiredState(1),
				desiredState(2),
				desiredState(3));

		targetPoseDisplacement.template head<3>() = qxyz.toRotationMatrix().eulerAngles(0, 1, 2);
		targetPoseDisplacement(3) = desiredState(4);
		targetPoseDisplacement(4) = desiredState(5);
		targetPoseDisplacement(5) = desiredState(6);

		targetVelocity(0) = desiredState(7);
		targetVelocity(1) = desiredState(8);
		targetVelocity(2) = desiredState(9);
		targetVelocity(3) = desiredState(10);
		targetVelocity(4) = desiredState(11);
		targetVelocity(5) = desiredState(12);
	}

	/**
	 * Transforms the target pose displacement to the cost function's desired state.
	 *
	 * @param [in] targetPoseDisplacementVelocity: The target base pose displacement and base local velocities.
	 * @param [out] desiredState: The cost function's desired state.
	 */
	static void toCostDesiredState(
				const scalar_array_t& targetPoseDisplacementVelocity,
				dynamic_vector_t& desiredState) {

		if (targetPoseDisplacementVelocity.size()<12)
			throw std::runtime_error("target command should have at least 12 elements.");

		auto deg2rad = [](const scalar_t& deg) { return (deg*M_PI/180.0); };

		desiredState.resize(targetPoseDisplacementVelocity.size()+1);

		// Orientation
		Eigen::Quaternion<scalar_t> qxyz =
				Eigen::AngleAxis<scalar_t>(
						deg2rad(targetPoseDisplacementVelocity[0]), Eigen::Vector3d::UnitX()) *  // roll
				Eigen::AngleAxis<scalar_t>(
						deg2rad(targetPoseDisplacementVelocity[1]), Eigen::Vector3d::UnitY()) *  // pitch
				Eigen::AngleAxis<scalar_t>(
						deg2rad(targetPoseDisplacementVelocity[2]), Eigen::Vector3d::UnitZ());   // yaw

		desiredState(0) = qxyz.w();
		desiredState(1) = qxyz.x();
		desiredState(2) = qxyz.y();
		desiredState(3) = qxyz.z();

		// Position
		desiredState(4) = targetPoseDisplacementVelocity[3];
		desiredState(5) = targetPoseDisplacementVelocity[4];
		desiredState(6) = targetPoseDisplacementVelocity[5];

		// Angular velocity
		desiredState(7) = targetPoseDisplacementVelocity[6];
		desiredState(8) = targetPoseDisplacementVelocity[7];
		desiredState(9) = targetPoseDisplacementVelocity[8];

		// Linear velocity
		desiredState(10) = targetPoseDisplacementVelocity[9];
		desiredState(11) = targetPoseDisplacementVelocity[10];
		desiredState(12) = targetPoseDisplacementVelocity[11];
	}

private:

};

} // end of namespace switched_model

#endif /* TARGETPOSECOMMAND_H_ */
