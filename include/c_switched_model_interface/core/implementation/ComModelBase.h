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
	Eigen::Matrix3d rotationMInverse = I.topLeftCorner<3,3>().inverse();
	Eigen::Matrix<double,6,6> I_inverse;
	I_inverse << rotationMInverse, Eigen::Matrix3d::Zero(),
			Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity()/I(5,5);

	return I_inverse * comMomentumJacobian(jointCoordinate);
}

} // end of namespace switched_model
