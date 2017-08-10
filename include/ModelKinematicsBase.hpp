/*
 * ModelKinematicsBase.hpp
 *
 *  Created on: Aug 2, 2017
 *      Author: sasutosh
 */

#ifndef MODELKINEMATICSBASE_HPP_
#define MODELKINEMATICSBASE_HPP_

#include <array>
#include <cmath>
#include <Eigen/Dense>

template< class DerivedClassType, size_t BASE_COORD_SIZE, size_t JOINT_COORD_SIZE >
class ModelKinematicsBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~ModelKinematicsBase(){};
	enum { LF=0,  RF=1,  LH=2,  RH=3,
		GENERALIZED_COORDINATE_SIZE = BASE_COORD_SIZE + JOINT_COORD_SIZE,
		BASE_COORDINATE_SIZE        = BASE_COORD_SIZE,
		JOINT_COORDINATE_SIZE       = JOINT_COORD_SIZE
	};

	typedef Eigen::Matrix<double,GENERALIZED_COORDINATE_SIZE,1> generalized_coordinate_t;
	typedef Eigen::Matrix<double,JOINT_COORDINATE_SIZE,1>       joint_coordinate_t;
	typedef Eigen::Matrix<double,BASE_COORDINATE_SIZE,1>        base_coordinate_t;

	/*
	 * gets a 18-by-1 generalized coordinate and calculates the rotation ...
	 */
	virtual void update(const generalized_coordinate_t& generalizedCoordinate);

	/*
	 * calculate the feet position in the Base frame
	 * 		+ single foot:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values (static function).
	 *		+ four feet:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values (static function).
	 */
	virtual void footPositionBaseFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);
	virtual void feetPositionsBaseFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	static void FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);
	static void FeetPositionsBaseFrame(const joint_coordinate_t& jointCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/*
	 * calculate the CoM position in the Base frame
	 */
	static void ComPositionBaseFrame(const joint_coordinate_t& jointCoordinate, Eigen::Vector3d& comPosition);

	/*
	 * calculate the feet position in the Origin frame
	 * 		+ single foot:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 *		+ four feet:
	 * 			 using the internal coordinate values set by update method.
	 * 			 using the input coordinate values.
	 */
	virtual void footPositionOriginFrame(const size_t& footIndex, Eigen::Vector3d& footPosition);
	virtual void footPositionOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);

	virtual void feetPositionsOriginFrame(std::array<Eigen::Vector3d,4>& feetPositions);

	virtual void feetPositionsOriginFrame(const generalized_coordinate_t& generalizedCoordinate,
			std::array<Eigen::Vector3d,4>& feetPositions);

	/*
	 * calculate foot Jacobian in Base frame
	 */
	virtual void footJacobainBaseFrame(const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain);

	static void FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain);

	/*
	 * calculate CoM Jacobian in Base frame
	 */
	static void ComJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& comJacobain);

	/*
	 * calculates the Jacobian matrix in the Inertia frame using rotation "i_R_b" from the Base frame to Inertia
	 * Frame and the Jacobian matrix expressed in the Base frame.
	 */
	static void FromBaseJacobianToInertiaJacobian(
			const Eigen::Matrix3d& i_R_b,
			const Eigen::Vector3d& b_r_point,
			const Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& b_J_point,
			Eigen::Matrix<double,6,GENERALIZED_COORDINATE_SIZE>& i_J_point);

	/*
	 * Origin to base rotation matrix
	 */
	Eigen::Matrix3d rotationMatrixOrigintoBase() const;

	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixOrigintoBase(const Eigen::DenseBase<Derived>& eulerAngles)
	{

		if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

		// inputs are the intrinsic rotation angles in RADIANTS
		double sinAlpha = sin(eulerAngles(0));
		double cosAlpha = cos(eulerAngles(0));
		double sinBeta  = sin(eulerAngles(1));
		double cosBeta  = cos(eulerAngles(1));
		double sinGamma = sin(eulerAngles(2));
		double cosGamma = cos(eulerAngles(2));

		Eigen::Matrix3d Rx, Ry, Rz;
		Rx << 1, 0, 0,					0, cosAlpha, sinAlpha,		0, -sinAlpha, cosAlpha;
		Ry << cosBeta, 0, -sinBeta,		0, 1, 0,		 			sinBeta, 0, cosBeta;
		Rz << cosGamma, sinGamma, 0, 	-sinGamma, cosGamma, 0,		0, 0, 1;

		return Rz*Ry*Rx;
	}

	template <typename Derived>
	static Eigen::Matrix3d RotationMatrixBasetoOrigin(const Eigen::DenseBase<Derived>& eulerAngles)
	{

		if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

		// inputs are the intrinsic rotation angles in RADIANTS
		double sinAlpha = sin(eulerAngles(0));
		double cosAlpha = cos(eulerAngles(0));
		double sinBeta  = sin(eulerAngles(1));
		double cosBeta  = cos(eulerAngles(1));
		double sinGamma = sin(eulerAngles(2));
		double cosGamma = cos(eulerAngles(2));

		Eigen::Matrix3d RxT, RyT, RzT;
		RxT << 1, 0, 0,					0, cosAlpha, -sinAlpha,		0, sinAlpha, cosAlpha;
		RyT << cosBeta, 0, sinBeta,		0, 1, 0,		 			-sinBeta, 0, cosBeta;
		RzT << cosGamma, -sinGamma, 0, 	sinGamma, cosGamma, 0,		0, 0, 1;

		return RxT*RyT*RzT;
	}

	/*
	 * calculates the skew matrix for vector cross product
	 */
	template <typename Derived>
	static Eigen::Matrix3d CrossProductMatrix(const Eigen::DenseBase<Derived>& in)
	{

		if (in.innerSize()!=3 || in.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

		Eigen::Matrix3d out;
		out <<   0.0,   -in(2), +in(1),
				+in(2),  0.0,   -in(0),
				-in(1), +in(0),  0.0;
		return out;
	}

protected:
	base_coordinate_t  qBase_;
	joint_coordinate_t qJoint_;
	Eigen::Matrix3d b_R_o_;

};

#include "implementation/ModelKinematicsBase.hpp"

#endif /* MODELKINEMATICSBASE_HPP_ */
