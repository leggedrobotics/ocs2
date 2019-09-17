/*
 * AnymalKinematics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#ifndef ANYMAL_ANYMALKINEMATICS_H_
#define ANYMAL_ANYMALKINEMATICS_H_

#include <iit/rbd/traits/TraitSelector.h>
#include <ocs2_switched_model_interface/core/KinematicsModelBase.h>

#include "ocs2_anymal_switched_model/generated/jacobians.h"
#include "ocs2_anymal_switched_model/generated/transforms.h"


namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalKinematics final : public switched_model::KinematicsModelBase<12, SCALAR_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::KinematicsModelBase<12, SCALAR_T> BASE;

	enum { LF=0,  RF=1,  LH=2,  RH=3 };

	typedef typename BASE::vector3d_t                vector3d_t;
	typedef typename BASE::matrix3d_t                matrix3d_t;
	typedef typename BASE::base_coordinate_t         base_coordinate_t;
	typedef typename BASE::joint_coordinate_t        joint_coordinate_t;
	typedef typename BASE::generalized_coordinate_t  generalized_coordinate_t;

	typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;

	/**
	 * Constructor needed for initialization
	 */
	AnymalKinematics() = default;

	/**
	 * Default destructor
	 */
	~AnymalKinematics() = default;

	/**
	 * Clone AnymalKinematics class.
	 */
	AnymalKinematics<SCALAR_T>* clone() const override;

	/**
	 * calculate foot position in Base frame
	 */
	void footPositionBaseFrame(const size_t& footIndex, vector3d_t& footPosition) override;

	/**
	 * calculate foot Jacobian in Base frame
	 */
	void footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<SCALAR_T,6,12>& footJacobian) override;

private:
	// RBD homogeneous transforms of feet
	typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LF_FOOT fr_trunk_X_fr_LF_foot_;
	typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RF_FOOT fr_trunk_X_fr_RF_foot_;
	typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_LH_FOOT fr_trunk_X_fr_LH_foot_;
	typename iit::ANYmal::tpl::HomogeneousTransforms<trait_t>::Type_fr_base_X_fr_RH_FOOT fr_trunk_X_fr_RH_foot_;

	// RBD Jacobian of feet
	typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LF_FOOT fr_trunk_J_fr_LF_foot_;
	typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RF_FOOT fr_trunk_J_fr_RF_foot_;
	typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_LH_FOOT fr_trunk_J_fr_LH_foot_;
	typename iit::ANYmal::tpl::Jacobians<trait_t>::Type_fr_base_J_fr_RH_FOOT fr_trunk_J_fr_RH_foot_;

};

}  // end of tpl namespace

using AnymalKinematics = tpl::AnymalKinematics<double>;

}  // end of anymal namespace


#include "implementation/AnymalKinematics.h"

#endif /* ANYMAL_ANYMALKINEMATICS_H_ */
