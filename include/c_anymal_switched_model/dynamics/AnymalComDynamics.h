/*
 * AnymalDynamics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: Jan Carius
 */

#ifndef ANYMAL_ANYMALCOMDYNAMICS_H_
#define ANYMAL_ANYMALCOMDYNAMICS_H_

#include <c_switched_model_interface/ComModelBase.h>
#include <CoM/CoMGenerated.h>

namespace anymal
{

class AnymalComDynamics : public ComModelBase<12>
{
public:

  enum { LF=0,  RF=1,  LH=2,  RH=3 };

  /**
	 * calculate CoM Position in Base frame
	 */
	Eigen::Vector3d comPositionBaseFrame(
			const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Inertia
	 */
	Eigen::Matrix<double, 6, 6> comInertia(
			const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate Com Homogeneous
	 */
	Eigen::Matrix<double,4,4> comHomogeneous(
			const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Inertia Derivative
	 */
	Eigen::Matrix<double,6,6> comInertiaDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);

	/**
	 * calculate CoM Momentum Jacobian
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobian(
			const Eigen::Matrix<double,12,1>& q);

	/**
	 * calculate CoM Momentum Jacobian Derivative
	 */
	Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);

	/**
	 * calculate CoM Velocity in Base Frame
	 */
	Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq);
};

}  // end of anymal namespace

#include "implementation/AnymalComDynamics.h"

#endif /* ANYMAL_ANYMALCOMDYNAMICS_H_ */
