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

template< class DerivedClassType, size_t JOINT_COORD_SIZE >
class ModelKinematicsBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~ModelKinematicsBase(){};
	enum { LF=0,  RF=1,  LH=2,  RH=3,
		BASE_COORDINATE_SIZE        = 6,
		JOINT_COORDINATE_SIZE       = JOINT_COORD_SIZE,
		GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORD_SIZE
	};

	typedef Eigen::Matrix<double,GENERALIZED_COORDINATE_SIZE,1> generalized_coordinate_t;
	typedef Eigen::Matrix<double,JOINT_COORDINATE_SIZE,1>       joint_coordinate_t;
	typedef Eigen::Matrix<double,BASE_COORDINATE_SIZE,1>        base_coordinate_t;

	/**
	 * gets a 18-by-1 generalized coordinate and calculates the rotation ...
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
	 */
	void footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);
	void feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	static void FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);
	static void FeetPositionsBaseFrame(const joint_coordinate_t& jointCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * calculate the CoM position in the Base frame
	 */
	static void ComPositionBaseFrame(const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition);

	/**
	 * calculate the feet position in the Origin frame
	 * 		+ single foot:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 *		+ four feet:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 */
	void footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);
	void footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);

	void feetPositionsOriginFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	void feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/**
	 * calculate foot Jacobian in Base frame
	 */
	void footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain);

	static void FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain);

	/**
	 * calculate CoM Jacobian in Base frame
	 */
	static void ComJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& comJacobain);

	/**
	 * calculates the Jacobian matrix in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
	 * Frame and the Jacobian matrix expressed in the Base frame.
	 */
	static void FromBaseJacobianToInertiaJacobian(
			const Eigen::Matrix3d& i_R_b,
			const Eigen::Vector3d& b_r_point,
			const Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& b_J_point,
			Eigen::Matrix<double,6,GENERALIZED_COORDINATE_SIZE>& i_J_point);

	/**
	 * Origin to base rotation matrix
	 */
	Eigen::Matrix3d rotationMatrixOrigintoBase() const;

	/**
	 * Origin to base rotation matrix
	 */
	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixOrigintoBase(const Eigen::DenseBase<Derived>& eulerAngles);

	/**
	 * Base to origin rotation matrix
	 */
	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixBasetoOrigin(const Eigen::DenseBase<Derived>& eulerAngles);

	/**
	 * Calculates the skew matrix for vector cross product
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
