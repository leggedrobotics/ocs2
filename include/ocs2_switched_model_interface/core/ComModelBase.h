/*
 * ComModelBase.h
 *
 *  Created on: Aug 10, 2017
 *      Author: farbod
 */

#ifndef COMMODELSBASE_H_
#define COMMODELSBASE_H_


#include <array>
#include <cmath>
#include <memory>
#include <Eigen/Dense>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * CoM Model Base Class
 * @tparam JOINT_COORD_SIZE
 */
template <size_t JOINT_COORD_SIZE, typename SCALAR_T=double>
class ComModelBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::unique_ptr<ComModelBase<JOINT_COORD_SIZE, SCALAR_T>> Ptr;

	typedef typename SwitchedModel<JOINT_COORD_SIZE, SCALAR_T>::generalized_coordinate_t  generalized_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE, SCALAR_T>::joint_coordinate_t        joint_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE, SCALAR_T>::base_coordinate_t         base_coordinate_t;

	typedef Eigen::Matrix<SCALAR_T, 3, 1> vector3d_t;
	typedef Eigen::Matrix<SCALAR_T, 3, 3> matrix3d_t;

	ComModelBase() = default;

	virtual ~ComModelBase() = default;

	/**
	 * Clone ComModelBase class.
	 */
	virtual ComModelBase<JOINT_COORD_SIZE, SCALAR_T>* clone() const = 0;

	/**
	 * Calculate CoM Jacobian in Base frame
	 *
	 * @param [in] jointCoordinate
	 * @param [out] comJacobain
	 */
	virtual Eigen::Matrix<SCALAR_T,6,JOINT_COORD_SIZE> comJacobainBaseFrame(
			const joint_coordinate_t& jointCoordinate);

	/**
	 * Calculate CoM Position in Base frame from Base origin
	 * @param [in] q
	 * @param [out] comPosition
	 */
	virtual vector3d_t comPositionBaseFrame(
			const joint_coordinate_t& q) = 0;

	/**
	 * Calculate CoM inertia tensor around CoM
	 *
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,6,6> comInertia(
			const Eigen::Matrix<SCALAR_T,12,1>& q) = 0;

	/**
	 * Total mass of robot
	 * @return mass in kg
	 */
	virtual SCALAR_T totalMass() const = 0;

	/**
	 * Calculate Com Homogeneous
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,4,4> comHomogeneous(
			const Eigen::Matrix<SCALAR_T,12,1>& q) = 0;

	/**
	 * Calculate CoM Inertia Derivative
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,6,6> comInertiaDerivative(
			const Eigen::Matrix<SCALAR_T,12,1>& q,
			const Eigen::Matrix<SCALAR_T,12,1>& dq) = 0;

	/**
	 * Calculate CoM Momentum Jacobian
	 * @param [in] q
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,6,12> comMomentumJacobian(
			const Eigen::Matrix<SCALAR_T,12,1>& q) = 0;

	/**
	 * Calculate CoM Momentum Jacobian Derivative
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,6,12> comMomentumJacobianDerivative(
			const Eigen::Matrix<SCALAR_T,12,1>& q,
			const Eigen::Matrix<SCALAR_T,12,1>& dq) = 0;

	/**
	 * Calculate CoM Velocity in Base Frame
	 * @param [in] q
	 * @param [in] dq
	 * @return
	 */
	virtual Eigen::Matrix<SCALAR_T,3,1> comVelocityInBaseFrame(
			const Eigen::Matrix<SCALAR_T,12,1>& q, const Eigen::Matrix<SCALAR_T,12,1>& dq) = 0;

	/**
	 * Calculate Base Pose from CoM pose for given joint coordinates
	 * The Base pose (basePose) consists of:
	 * 		+ Base orientation w.r.t origin frame (3-states)
	 * 		+ Base position w.r.t origin frame (3-states)
	 *
	 * @param [in]  qJoints
	 * @param [in]  comPose
	 * @param [out] basePose
	*/
	virtual void calculateBasePose(const joint_coordinate_t& qJoints,
			const base_coordinate_t& comPose, base_coordinate_t& basePose);

	/**
	 * Calculates the Base local velocities based on the current joint coordinates
	 * joint velocities, and CoM local velocities (comLocalVelocities).
	 * The Base local velocities (baseLocalVelocities) consists of angular and
	 * linear velocities in base frame (inertia frame coincide at Base frame)
	 * (6-states)
	 */
	void calculateBaseLocalVelocities(const joint_coordinate_t& qJoints,
			const joint_coordinate_t& dqJoints,
			const base_coordinate_t& comLocalVelocities,
			base_coordinate_t& baseLocalVelocities);
};

} //end of namespace switched_model

#include "implementation/ComModelBase.h"

#endif /* COMMODELSBASE_H_ */
