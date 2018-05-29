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
 * targetPoseDisplacement(0): Roll
 * targetPoseDisplacement(1): Pitch
 * targetPoseDisplacement(2): Yaw
 *
 * targetPoseDisplacement(3): X
 * targetPoseDisplacement(4): Y
 * targetPoseDisplacement(5): Z
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
 */

template <typename SCALAR_T>
class TargetPoseCommand
{
public:
	enum DIM {
		POSE_DIM_ = 6,
	};

	typedef SCALAR_T 									scalar_t;
	typedef std::vector<scalar_t> 						scalar_array_t;
	typedef Eigen::Matrix<scalar_t, POSE_DIM_, 1>		pose_vector_t;
	typedef Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>	cost_state_vector_t;

	/**
	 *
	 * @param goalPoseLimit
	 */
	TargetPoseCommand(
			const scalar_array_t& goalPoseLimit = scalar_array_t{45.0, 45.0, 360.0, 2.0, 1.0, 0.3})

	: goalPoseLimit_(goalPoseLimit)
	{
		if (goalPoseLimit_.size() != POSE_DIM_)
			throw std::runtime_error("The goal pose limit should be of size 6.");
	}

	~TargetPoseCommand() = default;

	/**
	 * Transforms the cost function's desired state to the target pose displacement.
	 *
	 * @param [in] desiredState: The cost function's desired state.
	 * @param [out] targetPoseDisplacement: The target pose displacement.
	 */
	static void toTargetPoseDisplacement(
			const cost_state_vector_t& desiredState,
			pose_vector_t& targetPoseDisplacement) {

		Eigen::Quaternion<scalar_t> qxyz(
				desiredState(0),
				desiredState(1),
				desiredState(2),
				desiredState(3));

		targetPoseDisplacement.template head<3>() = qxyz.toRotationMatrix().eulerAngles(0, 1, 2);
		targetPoseDisplacement(3) = desiredState(4);
		targetPoseDisplacement(4) = desiredState(5);
		targetPoseDisplacement(5) = desiredState(6);
	}

	/**
	 * Transforms the target pose displacement to the cost function's desired state.
	 * Moreover, it clamps the targetPoseDisplacement to its limits.
	 *
	 * @param [in] targetPoseDisplacement: The target pose displacement.
	 * @param [out] desiredState: The cost function's desired state.
	 * @return: The clamped targetPoseDisplacement.
	 */
	pose_vector_t toCostDesiredState(
				const pose_vector_t& targetPoseDisplacement,
				cost_state_vector_t& desiredState) {

		auto deg2rad = [](const scalar_t& deg) { return (deg*M_PI/180); };

		// limits
		pose_vector_t targetPoseDisplacementLimited;
		for (size_t i=0; i<POSE_DIM_; i++) {
			if (std::abs(targetPoseDisplacement(i)) > goalPoseLimit_[i])
				targetPoseDisplacementLimited(i) = (targetPoseDisplacement(i) > 0) ? goalPoseLimit_[i] : -goalPoseLimit_[i];
			else
				targetPoseDisplacementLimited(i) = targetPoseDisplacement(i);
		}  // end of i loop

		desiredState.resize(POSE_DIM_+1);

		// Orientation
		Eigen::Quaternion<scalar_t> qxyz =
				Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementLimited[0]), Eigen::Vector3d::UnitX()) *  // roll
				Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementLimited[1]), Eigen::Vector3d::UnitY()) *  // pitch
				Eigen::AngleAxis<scalar_t>(deg2rad(targetPoseDisplacementLimited[2]), Eigen::Vector3d::UnitZ());   // yaw

		desiredState(0) = qxyz.w();
		desiredState(1) = qxyz.x();
		desiredState(2) = qxyz.y();
		desiredState(3) = qxyz.z();

		// Position
		desiredState(4) = targetPoseDisplacementLimited[3];
		desiredState(5) = targetPoseDisplacementLimited[4];
		desiredState(6) = targetPoseDisplacementLimited[5];

		return targetPoseDisplacementLimited;
	}

	/**
	 * Gets the target pose displacement from command line. The first 3 input are the (x,y,z)
	 * position and the 2nd three inputs are (roll,pitch,yaw) orientation.
	 *
	 * @param [out] targetPoseDisplacement: The target pose displacement.
	 */
	void getCommandLineTargetPoseDisplacement(pose_vector_t& targetPoseDisplacement) {

		scalar_array_t commadInput(0);

		std::string line;
		std::getline(std::cin, line);
		std::istringstream stream(line);
		scalar_t in;
		while (stream >> in)
			commadInput.push_back(in);

		// if the size is greater than POSE_DIM_
		const size_t n = commadInput.size();
		if (n > POSE_DIM_)
			commadInput.erase(commadInput.begin()+POSE_DIM_, commadInput.end());
		else
			for (size_t i=n; i<POSE_DIM_; i++) {
				commadInput.push_back(0.0);
			}  // end of i loop

		// reversing the order of the position and orientation.
		for (size_t j=0; j<3; j++) {
			targetPoseDisplacement(j) = commadInput[3+j];
			targetPoseDisplacement(3+j) = commadInput[j];
		}
	}


private:
	scalar_array_t goalPoseLimit_;

	scalar_array_t commadInput_;
};

} // end of namespace switched_model

#endif /* TARGETPOSECOMMAND_H_ */
