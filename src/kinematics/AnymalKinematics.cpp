/*
 * AnymalKinematics.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#include <c_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal
{

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalKinematics::FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Vector3d& footPosition) {

	switch (footIndex) {
	case LF:
	{
		iit::ANYmal::HomogeneousTransforms::Type_fr_trunk_X_fr_LF_foot fr_trunk_X_fr_LF_foot;
		footPosition = fr_trunk_X_fr_LF_foot(jointCoordinate).topRightCorner<3,1>();
		break;
	}
	case RF:
	{
		iit::ANYmal::HomogeneousTransforms::Type_fr_trunk_X_fr_RF_foot fr_trunk_X_fr_RF_foot;
		footPosition = fr_trunk_X_fr_RF_foot(jointCoordinate).topRightCorner<3,1>();
		break;
	}
	case LH:
	{
		iit::ANYmal::HomogeneousTransforms::Type_fr_trunk_X_fr_LH_foot fr_trunk_X_fr_LH_foot;
		footPosition = fr_trunk_X_fr_LH_foot(jointCoordinate).topRightCorner<3,1>();
		break;
	}
	case RH:
	{
		iit::ANYmal::HomogeneousTransforms::Type_fr_trunk_X_fr_RH_foot fr_trunk_X_fr_RH_foot;
		footPosition = fr_trunk_X_fr_RH_foot(jointCoordinate).topRightCorner<3,1>();
		break;
	}
	default:
		std::runtime_error("Not defined foot index.");
		break;
	}

}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalKinematics::FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
		const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain) {

	footJacobain.setZero();

	switch (footIndex) {
	case LF:
	{
		iit::ANYmal::Jacobians::Type_fr_trunk_J_fr_LF_foot fr_trunk_J_fr_LF_foot;
		footJacobain.block<6,3>(0,0) = fr_trunk_J_fr_LF_foot(jointCoordinate);
		break;
	}
	case RF:
	{
		iit::ANYmal::Jacobians::Type_fr_trunk_J_fr_RF_foot fr_trunk_J_fr_RF_foot;
		footJacobain.block<6,3>(0,3) = fr_trunk_J_fr_RF_foot(jointCoordinate);
		break;
	}
	case LH:
	{
		iit::ANYmal::Jacobians::Type_fr_trunk_J_fr_LH_foot fr_trunk_J_fr_LH_foot;
		footJacobain.block<6,3>(0,6) = fr_trunk_J_fr_LH_foot(jointCoordinate);
		break;
	}
	case RH:
	{
		iit::ANYmal::Jacobians::Type_fr_trunk_J_fr_RH_foot fr_trunk_J_fr_RH_foot;
		footJacobain.block<6,3>(0,9) = fr_trunk_J_fr_RH_foot(jointCoordinate);
		break;
	}
	default:
	{
		std::runtime_error("Not defined foot index.");
		break;
	}
	}
}


}  // end of namespace anymal



