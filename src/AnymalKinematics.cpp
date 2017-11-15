/*
 * AnymalKinematics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>

namespace anymal {

std::shared_ptr<AnymalKinematics::Base> AnymalKinematics::clone() const {
	return std::allocate_shared< AnymalKinematics, Eigen::aligned_allocator<AnymalKinematics> > (
					Eigen::aligned_allocator<AnymalKinematics>(), *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalKinematics::footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition){

	switch (footIndex) {
	case LF:
	{
		footPosition = fr_trunk_X_fr_LF_foot_(qJoint_).topRightCorner<3,1>();
		break;
	}
	case RF:
	{
		footPosition = fr_trunk_X_fr_RF_foot_(qJoint_).topRightCorner<3,1>();
		break;
	}
	case LH:
	{
		footPosition = fr_trunk_X_fr_LH_foot_(qJoint_).topRightCorner<3,1>();
		break;
	}
	case RH:
	{
		footPosition = fr_trunk_X_fr_RH_foot_(qJoint_).topRightCorner<3,1>();
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
void AnymalKinematics::footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,12>& footJacobian){

	footJacobian.setZero();

	switch (footIndex) {
	case LF:
	{
		footJacobian.block<6,3>(0,0) = fr_trunk_J_fr_LF_foot_(qJoint_);
		break;
	}
	case RF:
	{
		footJacobian.block<6,3>(0,3) = fr_trunk_J_fr_RF_foot_(qJoint_);
		break;
	}
	case LH:
	{
		footJacobian.block<6,3>(0,6) = fr_trunk_J_fr_LH_foot_(qJoint_);
		break;
	}
	case RH:
	{
		footJacobian.block<6,3>(0,9) = fr_trunk_J_fr_RH_foot_(qJoint_);
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
