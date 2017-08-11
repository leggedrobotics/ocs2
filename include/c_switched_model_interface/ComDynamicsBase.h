/*
 * ComDynamicsBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: asutosh
 */

#ifndef COMDYNAMICSBASE_H_
#define COMDYNAMICSBASE_H_


#include <array>
#include <cmath>
#include <Eigen/Dense>

#include "SwitchedModel.h"

template< class DerivedClassType, size_t JOINT_COORD_SIZE >
class ComDynamicsBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::generalized_coordinate_t generalized_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t base_coordinate_t;

	/**
	 * calculate CoM Jacobian in Base frame
	 */
	virtual void comJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			Eigen::Matrix<double,6,JOINT_COORD_SIZE>& comJacobain);

	/**
	 * calculate CoM Position in Base frame
	 */
	static void ComPositionBaseFrame(
			const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition);

	/**
	 * calculate CoM Inertia
	 */
	virtual Eigen::Matrix<double,6,6> comInertia(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * calculate Com Homogeneous
	 */
	virtual Eigen::Matrix<double,4,4> comHomogeneous(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * calculate CoM Inertia Derivative
	 */
	virtual Eigen::Matrix<double,6,6> comInertiaDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq) = 0;

	/**
	 * calculate CoM Momentum Jacobian
	 */
	virtual Eigen::Matrix<double,6,12> comMomentumJacobian(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * calculate CoM Momentum Jacobian Derivative
	 */
	virtual Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq) = 0;

	/**
	 * calculate CoM Velocity in Base Frame
	 */
	virtual Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<double,12,1>& q, const Eigen::Matrix<double,12,1>& dq) = 0;

};

#include "implementation/ComDynamicsBase.h"

#endif /* COMDYNAMICSBASE_H_ */
