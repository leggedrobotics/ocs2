/*
 * ComDynamicsBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: asutosh
 */

#ifndef COMDYNAMICSBASE_H_
#define COMDYNAMICSBASE_H_

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t JOINT_COORD_SIZE >
void ComDynamicsBase<DerivedClassType, JOINT_COORD_SIZE>::comJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
		Eigen::Matrix<double,6,JOINT_COORD_SIZE>& comJacobain) {

	// CoM jacobian in the Base frame around CoM
	Eigen::Matrix<double,6,6> I = comInertia(jointCoordinate);
	comJacobain = I.inverse() * comMomentumJacobian(jointCoordinate);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template< class DerivedClassType, size_t JOINT_COORD_SIZE >
void ComDynamicsBase<DerivedClassType, JOINT_COORD_SIZE>::ComPositionBaseFrame(const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition)
{
	DerivedClassType::ComPositionBaseFrame(jointCoordinate, comPosition);
}

#endif /* COMDYNAMICSBASE_H_ */
