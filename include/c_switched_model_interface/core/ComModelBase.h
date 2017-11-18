/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod, asutosh
 */

#ifndef COMMODELSBASE_H_
#define COMMODELSBASE_H_


#include <array>
#include <cmath>
#include <memory>
#include <Eigen/Dense>

#include "SwitchedModel.h"

namespace switched_model {

/**
 * CoM Model Base Class
 * @tparam JOINT_COORD_SIZE
 */
template <size_t JOINT_COORD_SIZE>
class ComModelBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<ComModelBase<JOINT_COORD_SIZE>> Ptr;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::generalized_coordinate_t generalized_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t base_coordinate_t;


	ComModelBase() {}

	virtual ~ComModelBase() {}

	/**
	 * Clone ComModelBase class.
	 */
	virtual std::shared_ptr<ComModelBase<JOINT_COORD_SIZE>> clone() const = 0;

	/**
	 * Calculate CoM Jacobian in Base frame
	 *
	 * @param [in] jointCoordinate
	 * @param [out] comJacobain
	 */
	virtual Eigen::Matrix<double,6,JOINT_COORD_SIZE> comJacobainBaseFrame(
			const joint_coordinate_t& jointCoordinate);

	/**
	 * Calculate CoM Position in Base frame from Base origin
	 * @param [in] q
	 * @param [out] comPosition
	 */
	virtual Eigen::Vector3d comPositionBaseFrame(
			const joint_coordinate_t& q) = 0;

	/**
	 * Calculate CoM inertia tensor around CoM
	 *
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<double,6,6> comInertia(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * Calculate Com Homogeneous
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<double,4,4> comHomogeneous(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * Calculate CoM Inertia Derivative
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<double,6,6> comInertiaDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq) = 0;

	/**
	 * Calculate CoM Momentum Jacobian
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<double,6,12> comMomentumJacobian(
			const Eigen::Matrix<double,12,1>& q) = 0;

	/**
	 * Calculate CoM Momentum Jacobian Derivative
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<double,6,12> comMomentumJacobianDerivative(
			const Eigen::Matrix<double,12,1>& q,
			const Eigen::Matrix<double,12,1>& dq) = 0;

	/**
	 * Calculate CoM Velocity in Base Frame
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<double,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<double,12,1>& q, const Eigen::Matrix<double,12,1>& dq) = 0;

};

} //end of namespace switched_model

#include "implementation/ComModelBase.h"

#endif /* COMMODELSBASE_H_ */
