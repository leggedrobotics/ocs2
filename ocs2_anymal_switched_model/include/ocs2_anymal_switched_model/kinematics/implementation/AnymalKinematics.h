/*
 * AnymalKinematics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/kinematics/AnymalKinematics.h"

namespace anymal {
namespace tpl {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
AnymalKinematics<SCALAR_T>* AnymalKinematics<SCALAR_T>::clone() const {

	return new AnymalKinematics<SCALAR_T>(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
void AnymalKinematics<SCALAR_T>::footPositionBaseFrame(
		const size_t& footIndex,
		vector3d_t& footPosition){

	switch (footIndex) {
	case LF:
	{
		footPosition = fr_trunk_X_fr_LF_foot_(BASE::qJoint_).template topRightCorner<3,1>();
		break;
	}
	case RF:
	{
		footPosition = fr_trunk_X_fr_RF_foot_(BASE::qJoint_).template topRightCorner<3,1>();
		break;
	}
	case LH:
	{
		footPosition = fr_trunk_X_fr_LH_foot_(BASE::qJoint_).template topRightCorner<3,1>();
		break;
	}
	case RH:
	{
		footPosition = fr_trunk_X_fr_RH_foot_(BASE::qJoint_).template topRightCorner<3,1>();
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
template <typename SCALAR_T>
void AnymalKinematics<SCALAR_T>::footJacobainBaseFrame(
		const size_t& footIndex, Eigen::Matrix<SCALAR_T,6,12>& footJacobian){

	footJacobian.setZero();

	switch (footIndex) {
	case LF:
	{
		footJacobian.template block<6,3>(0,0) = fr_trunk_J_fr_LF_foot_(BASE::qJoint_);
		break;
	}
	case RF:
	{
		footJacobian.template block<6,3>(0,3) = fr_trunk_J_fr_RF_foot_(BASE::qJoint_);
		break;
	}
	case LH:
	{
		footJacobian.template block<6,3>(0,6) = fr_trunk_J_fr_LH_foot_(BASE::qJoint_);
		break;
	}
	case RH:
	{
		footJacobian.template block<6,3>(0,9) = fr_trunk_J_fr_RH_foot_(BASE::qJoint_);
		break;
	}
	default:
	{
		std::runtime_error("Not defined foot index.");
		break;
	}
	}
}

}  // end of tpl namespace
}  // end of namespace anymal
