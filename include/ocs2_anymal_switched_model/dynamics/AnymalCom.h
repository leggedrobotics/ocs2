/*
 * AnymalCom.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

#ifndef ANYMAL_ANYMALCOM_H_
#define ANYMAL_ANYMALCOM_H_

#include <c_switched_model_interface/ComModelBase.h>

#include <iit/robots/anymal/inertia_properties.h>
#include <iit/robots/anymal/transforms.h>
#include <iit/robots/anymal/jsim.h>
#include <iit/robots/anymal/miscellaneous.h>

namespace anymal
{

class AnymalCom : public ComModelBase<12>
{
public:

	enum { LF=0,  RF=1,  LH=2,  RH=3 };

	/**
	 * Constructor needed for initialization
	 */
	AnymalCom();

	/**
	 * calculate CoM Position in Base frame
	 */
	Eigen::Vector3d comPositionBaseFrame(const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate homogeneous transformation base -> CoM
	 */
	Eigen::Matrix<double,4,4> comHomogeneous(const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Velocity in Base Frame
	 */
	Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);

	/**
	 * calculate CoM Inertia
	 */
	Eigen::Matrix<double, 6, 6> comInertia(const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Inertia Derivative
	 */
	Eigen::Matrix<double,6,6> comInertiaDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);

	/**
	 * calculate CoM Momentum Jacobian
	 * i.e. p_com = J_p_com(q) * dq
	 * Note: excluding the contribution of the trunk inertia
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobian(
			const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Momentum Jacobian Derivative
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);



private:

	iit::ANYmal::dyn::InertiaProperties inertiaProperties_;
	iit::ANYmal::HomogeneousTransforms homTransforms_;
	iit::ANYmal::ForceTransforms forceTransforms_;
	iit::ANYmal::dyn::JSIM jointSpaceInertiaMatrix_;
};

}  // end of anymal namespace

#endif /* ANYMAL_ANYMALCOM_H_ */
