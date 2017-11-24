/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: asutosh
 */

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
Eigen::Matrix<double,6,JOINT_COORD_SIZE> ComModelBase<JOINT_COORD_SIZE>::comJacobainBaseFrame(
		const joint_coordinate_t& jointCoordinate) {

	// CoM jacobian in the Base frame around CoM
	Eigen::Matrix<double,6,6> I = comInertia(jointCoordinate);
	Eigen::Matrix<double,6,6> I_inverse = I.ldlt().solve(Eigen::MatrixXd::Identity(6,6));

	return I_inverse * comMomentumJacobian(jointCoordinate);
}

template <size_t JOINT_COORD_SIZE>
void ComModelBase<JOINT_COORD_SIZE>::calculateBasePose(const joint_coordinate_t& qJoints,
		const base_coordinate_t& comPose,
		base_coordinate_t& basePose) {

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	Eigen::Matrix3d o_R_b = RotationMatrixBasetoOrigin(comPose.template head<3>());

	// base to CoM displacement in the CoM frame
	Eigen::Vector3d com_base2CoM = comPositionBaseFrame(qJoints);

	// base coordinate
	basePose.template head<3>() = comPose.template segment<3>(0);
	basePose.template tail<3>() = comPose.template segment<3>(3) - o_R_b * com_base2CoM;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComModelBase<JOINT_COORD_SIZE>::calculateBaseLocalVelocities(const joint_coordinate_t& qJoints,
		const joint_coordinate_t& dqJoints,
		const base_coordinate_t& comLocalVelocities,
		base_coordinate_t& baseLocalVelocities) {

	// base to CoM displacement in the CoM frame
	Eigen::Vector3d com_base2CoM = comPositionBaseFrame(qJoints);

	// CoM Jacobin in the Base frame
	Eigen::Matrix<double,6,12> b_comJacobain = comJacobainBaseFrame(qJoints);

	// local velocities of Base (com_W_b)
	baseLocalVelocities.template head<3>() = comLocalVelocities.template head<3>() - b_comJacobain.template topRows<3>()*dqJoints;
	baseLocalVelocities.template tail<3>() = comLocalVelocities.template tail<3>() - b_comJacobain.template bottomRows<3>()*dqJoints
			+ com_base2CoM.cross(baseLocalVelocities.template head<3>());

}

} // end of namespace switched_model
