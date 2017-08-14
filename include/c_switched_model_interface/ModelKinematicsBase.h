/*
 * ModelKinematicsBase.h
 *
 *  Created on: Aug 2, 2017
 *      Author: sasutosh
 */

#ifndef MODELKINEMATICSBASE_H_
#define MODELKINEMATICSBASE_H_

#include <array>
#include <cmath>
#include <Eigen/Dense>

#include "SwitchedModel.h"

/**
 * ModelKinematics Base Class
 * @tparam DerivedClassType
 * @tparam JOINT_COORD_SIZE
 */
template< class DerivedClassType, size_t JOINT_COORD_SIZE >
class ModelKinematicsBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~ModelKinematicsBase(){};

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::generalized_coordinate_t generalized_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t base_coordinate_t;

	/**
	 * Gets a 18-by-1 generalized coordinate and calculates the rotation ...
	 * @param [in] generalizedCoordinate
	 */
	virtual void update(const generalized_coordinate_t& generalizedCoordinate);

	/**
	 * calculate the feet position in the Base frame
	 * 		+ single foot:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values (static function).
	 *		+ four feet:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values (static function).
	 * @param [in] footIndex
	 * @param [out] footPosition
	 */
	void footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);

	/**
	 * Calculate the feet position in the Base frame
	 * @param [out] feetPositions
	 */
	void feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * Calculate the feet position in the Base frame
	 * @param jointCoordinate
	 * @param footIndex
	 * @param footPosition
	 */
	static void FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);

	/**
	 * Calculate the feet position in the Base frame
	 * @param jointCoordinate
	 * @param feetPositions
	 */
	static void FeetPositionsBaseFrame(const joint_coordinate_t& jointCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * Calculate the feet position in the Origin frame
	 * 		+ single foot:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 *		+ four feet:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 * @param [in] footIndex
	 * @param [out] footPosition
	 */
	void footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);

	/**
	 * Calculate feet Positions in Origin Frame
	 * @param [in] generalizedCoordinate
	 * @param [in] footIndex
	 * @param [out] footPosition
	 */
	void footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);

	/**
	 * Calculate feet Positions in Origin Frame
	 * @param feetPositions
	 */
	void feetPositionsOriginFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * Calculate feet Positions in Origin Frame
	 * @param generalizedCoordinate
	 * @param feetPositions
	 */
	void feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * Calculate foot Jacobian in Base frame
	 * @param [in] footIndex
	 * @param [out] footJacobain
	 */
	void footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORD_SIZE>& footJacobain);

	/**
	 * Calculate foot Jacobain in Base Frame
	 * @param [in] jointCoordinate
	 * @param [in] footIndex
	 * @param [out] footJacobain
	 */
	static void FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORD_SIZE>& footJacobain);

	/**
	 * calculates the Jacobian matrix in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
	 * Frame and the Jacobian matrix expressed in the Base frame.
	 * @param [in] i_R_b
	 * @param [in] b_r_point
	 * @param [in] b_J_point
	 * @param [out] i_J_point
	 */
	static void FromBaseJacobianToInertiaJacobian(
			const Eigen::Matrix3d& i_R_b,
			const Eigen::Vector3d& b_r_point,
			const Eigen::Matrix<double,6,JOINT_COORD_SIZE>& b_J_point,
			Eigen::Matrix<double,6,JOINT_COORD_SIZE + 6>& i_J_point);

	/**
	 * Origin to base rotation matrix
	 * @return
	 */
	Eigen::Matrix3d rotationMatrixOrigintoBase() const;

	/**
	 * Origin to base rotation matrix
	 * @tparam Derived
	 * @param [in] eulerAngles
	 * @return
	 */
	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixOrigintoBase(const Eigen::DenseBase<Derived>& eulerAngles);

	/**
	 * Base to origin rotation matrix
	 * @tparam Derived
	 * @param [in] eulerAngles
	 * @return
	 */
	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixBasetoOrigin(const Eigen::DenseBase<Derived>& eulerAngles);

	/**
	 * Calculates the skew matrix for vector cross product
	 * @tparam Derived
	 * @param [in] in
	 * @return
	 */
	template <typename Derived>
	static Eigen::Matrix3d CrossProductMatrix(const Eigen::DenseBase<Derived>& in);

protected:
	base_coordinate_t  qBase_;
	joint_coordinate_t qJoint_;
	Eigen::Matrix3d b_R_o_;

};

#include "implementation/ModelKinematicsBase.h"

#endif /* MODELKINEMATICSBASE_H_ */
