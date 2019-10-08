/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

namespace switched_model {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
Eigen::Matrix<SCALAR_T,6,JOINT_COORD_SIZE> ComModelBase<JOINT_COORD_SIZE, SCALAR_T>::comJacobainBaseFrame(
		const joint_coordinate_t& jointCoordinate) {

	// CoM jacobian in the Base frame around CoM
	Eigen::Matrix<SCALAR_T,6,6> I = comInertia(jointCoordinate);
	Eigen::Matrix<SCALAR_T,6,6> I_inverse;
	matrix3d_t rotationMInverse = I.template topLeftCorner<3,3>().inverse();
	I_inverse << rotationMInverse,    matrix3d_t::Zero(),
			     matrix3d_t::Zero(),  (1/I(5,5))*matrix3d_t::Identity();

	return I_inverse * comMomentumJacobian(jointCoordinate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void ComModelBase<JOINT_COORD_SIZE, SCALAR_T>::calculateBasePose(
		const joint_coordinate_t& qJoints,
		const base_coordinate_t& comPose,
		base_coordinate_t& basePose) {

	// Rotation matrix from Base frame (or the coincided frame world frame) to Origin frame (global world).
	matrix3d_t o_R_b = RotationMatrixBasetoOrigin<SCALAR_T>(comPose.template head<3>());

	// base to CoM displacement in the CoM frame
	vector3d_t com_base2CoM = comPositionBaseFrame(qJoints);

	// base coordinate
	basePose.template head<3>() = comPose.template segment<3>(0);
	basePose.template tail<3>() = comPose.template segment<3>(3) - o_R_b * com_base2CoM;
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE, typename SCALAR_T>
void ComModelBase<JOINT_COORD_SIZE, SCALAR_T>::calculateBaseLocalVelocities(
		const joint_coordinate_t& qJoints,
		const joint_coordinate_t& dqJoints,
		const base_coordinate_t& comLocalVelocities,
		base_coordinate_t& baseLocalVelocities) {

	// base to CoM displacement in the CoM frame
	vector3d_t com_base2CoM = comPositionBaseFrame(qJoints);

	// CoM Jacobian in the Base frame
	Eigen::Matrix<SCALAR_T,6,JOINT_COORD_SIZE> b_comJacobain = comJacobainBaseFrame(qJoints);

	// local velocities of Base (com_W_b)
	baseLocalVelocities.template head<3>() = comLocalVelocities.template head<3>() - b_comJacobain.template topRows<3>()*dqJoints;
	baseLocalVelocities.template tail<3>() = comLocalVelocities.template tail<3>() - b_comJacobain.template bottomRows<3>()*dqJoints
			+ com_base2CoM.cross(baseLocalVelocities.template head<3>());

}

} // end of namespace switched_model
