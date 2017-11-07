/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: asutosh
 */

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <size_t JOINT_COORD_SIZE>
void ComModelBase<DerivedClassType, JOINT_COORD_SIZE>::comJacobainBaseFrame(
		const joint_coordinate_t& jointCoordinate,
		Eigen::Matrix<double,6,JOINT_COORD_SIZE>& comJacobain) {

	// CoM jacobian in the Base frame around CoM
	Eigen::Matrix<double,6,6> I = comInertia(jointCoordinate);
	comJacobain = I.inverse() * comMomentumJacobian(jointCoordinate);
}

