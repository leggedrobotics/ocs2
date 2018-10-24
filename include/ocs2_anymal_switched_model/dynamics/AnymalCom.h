/*
 * AnymalCom.h
 *
 *  Created on: Aug 11, 2018
 *      Author: farbod
 */

#ifndef ANYMAL_ANYMALCOM_H_
#define ANYMAL_ANYMALCOM_H_

#include <ocs2_robotic_examples/rbd_libraries/robcogen/iit/rbd/traits/TraitSelector.h>

#include <ocs2_anymal_switched_model/generated/inertia_properties.h>
#include <ocs2_anymal_switched_model/generated/transforms.h>
#include <ocs2_anymal_switched_model/generated/jsim.h>
#include <ocs2_anymal_switched_model/generated/miscellaneous.h>
#include <ocs2_switched_model_interface/core/ComModelBase.h>

namespace anymal {
namespace tpl {

template <typename SCALAR_T>
class AnymalCom : public switched_model::ComModelBase<12, SCALAR_T>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum { LF=0,  RF=1,  LH=2,  RH=3 };

	typedef switched_model::ComModelBase<12, SCALAR_T> BASE;

	typedef typename BASE::vector3d_t         vector3d_t;
	typedef typename BASE::matrix3d_t         matrix3d_t;
	typedef typename BASE::joint_coordinate_t joint_coordinate_t;

	typedef typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait trait_t;

	/**
	 * Constructor needed for initialization
	 */
	AnymalCom();

	/**
	 * Default destructor
	 */
	~AnymalCom() = default;

	/**
	 * Clone AnymalCom class.
	 */
	AnymalCom<SCALAR_T>* clone() const override;

	/**
	 * Set default joint configuration. Updates the CoM position and inertia
	 */
	void setJointConfiguration(const joint_coordinate_t& q);

	/**
	 * calculate CoM Position in Base frame for given joint q
	 */
	vector3d_t comPositionBaseFrame(const joint_coordinate_t& q) override;

	/**
	  calculate CoM Position in Base frame for default q
	 */
	vector3d_t comPositionBaseFrame() {return comPositionBaseFrame_;}

	/**
	 * calculate homogeneous transformation base -> CoM for given q
	 */
	Eigen::Matrix<SCALAR_T,4,4> comHomogeneous(const joint_coordinate_t& q) override;

	/**
	 * calculate homogeneous transformation base -> CoM for default q
	 */
	Eigen::Matrix<SCALAR_T,4,4> comHomogeneous();

	/**
	 * calculate CoM Velocity in Base Frame
	 */
	Eigen::Matrix<SCALAR_T,3,1> comVelocityInBaseFrame(
			const joint_coordinate_t& q,
			const joint_coordinate_t& dq) override;

	/**
	 * calculate CoM Inertia for given q
	 */
	Eigen::Matrix<SCALAR_T, 6, 6> comInertia(const joint_coordinate_t& q) override;

	/**
	 * Total mass of robot
	 * @return mass in kg
	 */
	SCALAR_T totalMass() const override;

	/**
	 * calculate CoM Inertia for default q
	 */
	Eigen::Matrix<SCALAR_T, 6, 6> comInertia() {return comInertia_;}

	/**
	 * calculate CoM Inertia Derivative
	 */
	Eigen::Matrix<SCALAR_T,6,6> comInertiaDerivative(
			const joint_coordinate_t& q,
			const joint_coordinate_t& dq) override;

	/**
	 * calculate CoM Momentum Jacobian
	 * i.e. p_com = J_p_com(q) * dq
	 * Note: excluding the contribution of the trunk inertia
	 */
	Eigen::Matrix<SCALAR_T,6,12> comMomentumJacobian(const joint_coordinate_t& q) override;

	/**
	 * calculate CoM Momentum Jacobian Derivative
	 */
	Eigen::Matrix<SCALAR_T,6,12> comMomentumJacobianDerivative(
			const joint_coordinate_t& q,
			const joint_coordinate_t& dq) override;

private:

	iit::ANYmal::dyn::tpl::InertiaProperties<trait_t>  inertiaProperties_;
	iit::ANYmal::tpl::HomogeneousTransforms<trait_t>   homTransforms_;
	iit::ANYmal::tpl::ForceTransforms<trait_t>         forceTransforms_;
	iit::ANYmal::dyn::tpl::JSIM<trait_t>               jointSpaceInertiaMatrix_;

	// cached values for current default joint configuration
	vector3d_t comPositionBaseFrame_;
	Eigen::Matrix<SCALAR_T,6,6> comInertia_;

};

}  // end of tpl namespace

using AnymalCom = tpl::AnymalCom<double>;

}  // end of anymal namespace

#include "implementation/AnymalCom.h"

#endif /* ANYMAL_ANYMALCOM_H_ */
