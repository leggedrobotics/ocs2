/*
 * SwitchedModel.h
 *
 *  Created on: Aug 11, 2017
 *      Author: asutosh
 */

#ifndef SWITCHEDMODEL_H_
#define SWITCHEDMODEL_H_

#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace switched_model {

/**
 * General typedefs and enums for all Base classes
 * @tparam JOINT_COORD_SIZE
 */
template <size_t JOINT_COORD_SIZE, typename SCALAR_T=double>
class SwitchedModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Enumerations defining base, joint and generalized coordinate size
	 */
	enum
	{
		NUM_CONTACT_POINTS			= 4,
		BASE_COORDINATE_SIZE        = 6,
		JOINT_COORDINATE_SIZE       = JOINT_COORD_SIZE,
		GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORD_SIZE
	};

	typedef std::array<bool, NUM_CONTACT_POINTS> contact_flag_t;

	typedef Eigen::Matrix<SCALAR_T, GENERALIZED_COORDINATE_SIZE,1> generalized_coordinate_t;
	typedef Eigen::Matrix<SCALAR_T, JOINT_COORDINATE_SIZE,1>       joint_coordinate_t;
	typedef Eigen::Matrix<SCALAR_T, BASE_COORDINATE_SIZE,1>        base_coordinate_t;

  	enum class FeetEnum { LF, RF, LH, RH };
};

/**
 * Whether to use Inertia Matrix derived
 * @return
 */
inline bool useInertiaMatrixDerivate() {
	return false;
}

/**
 * Origin to base rotation matrix
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T=double, typename Derived>
inline Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixOrigintoBase(const Eigen::DenseBase<Derived>& eulerAngles) {

	if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)
		throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	// inputs are the intrinsic rotation angles in RADIANTS
	SCALAR_T sinAlpha = sin(eulerAngles(0));
	SCALAR_T cosAlpha = cos(eulerAngles(0));
	SCALAR_T sinBeta  = sin(eulerAngles(1));
	SCALAR_T cosBeta  = cos(eulerAngles(1));
	SCALAR_T sinGamma = sin(eulerAngles(2));
	SCALAR_T cosGamma = cos(eulerAngles(2));

	Eigen::Matrix<SCALAR_T, 3, 3> Rx, Ry, Rz;
	Rx << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0),					SCALAR_T(0), cosAlpha, sinAlpha,		SCALAR_T(0), -sinAlpha, cosAlpha;
	Ry << cosBeta, SCALAR_T(0), -sinBeta,		SCALAR_T(0), SCALAR_T(1), SCALAR_T(0),		 			sinBeta, SCALAR_T(0), cosBeta;
	Rz << cosGamma, sinGamma, SCALAR_T(0), 	-sinGamma, cosGamma, SCALAR_T(0),		SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);

	return Rz*Ry*Rx;
}

/**
 * Base to origin rotation matrix
 * @tparam Derived
 * @param [in] eulerAngles
 * @return
 */
template <typename SCALAR_T=double, typename Derived>
inline Eigen::Matrix<SCALAR_T, 3, 3> RotationMatrixBasetoOrigin(const Eigen::DenseBase<Derived>& eulerAngles) {

	if (eulerAngles.innerSize()!=3 || eulerAngles.outerSize()!=1)
		throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	// inputs are the intrinsic rotation angles in RADIANTS
	SCALAR_T sinAlpha = sin(eulerAngles(0));
	SCALAR_T cosAlpha = cos(eulerAngles(0));
	SCALAR_T sinBeta  = sin(eulerAngles(1));
	SCALAR_T cosBeta  = cos(eulerAngles(1));
	SCALAR_T sinGamma = sin(eulerAngles(2));
	SCALAR_T cosGamma = cos(eulerAngles(2));

	Eigen::Matrix<SCALAR_T, 3, 3> RxT, RyT, RzT;
	RxT << SCALAR_T(1), SCALAR_T(0), SCALAR_T(0),					SCALAR_T(0), cosAlpha, -sinAlpha,		SCALAR_T(0), sinAlpha, cosAlpha;
	RyT << cosBeta, SCALAR_T(0), sinBeta,		SCALAR_T(0), SCALAR_T(1), SCALAR_T(0),		 			-sinBeta, SCALAR_T(0), cosBeta;
	RzT << cosGamma, -sinGamma, SCALAR_T(0), 	sinGamma, cosGamma, SCALAR_T(0),		SCALAR_T(0), SCALAR_T(0), SCALAR_T(1);

	return RxT*RyT*RzT;
}

/**
 * Calculates the skew matrix for vector cross product
 * @tparam Derived
 * @param [in] in
 * @return
 */
template <typename SCALAR_T=double, typename Derived>
inline Eigen::Matrix<SCALAR_T, 3, 3> CrossProductMatrix(const Eigen::DenseBase<Derived>& in) {

	if (in.innerSize()!=3 || in.outerSize()!=1)  throw std::runtime_error("Input argument should be a 3-by-1 vector.");

	Eigen::Matrix<SCALAR_T, 3, 3> out;
	out <<   SCALAR_T(0.0),   -in(2), +in(1),
			+in(2),  SCALAR_T(0.0),   -in(0),
			-in(1), +in(0),  SCALAR_T(0.0);
	return out;
}

/*
 * @brief Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
 * @param[in] eulerAngles: euler angles in xyz convention
 * @return M: matrix that does the transformation
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> AngularVelocitiesToEulerAngleDerivativesMatrix (const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles){

	Eigen::Matrix<SCALAR_T, 3, 3> M;
	SCALAR_T sinPsi = sin(eulerAngles(2));
	SCALAR_T cosPsi = cos(eulerAngles(2));
	SCALAR_T sinTheta = sin(eulerAngles(1));
	SCALAR_T cosTheta = cos(eulerAngles(1));

	M << 	cosPsi/cosTheta,          -sinPsi/cosTheta,          SCALAR_T(0),
			sinPsi, 				   cosPsi,                   SCALAR_T(0),
			-cosPsi*sinTheta/cosTheta,  sinTheta*sinPsi/cosTheta, SCALAR_T(1);

	return M;
}

} // end of namespace switched_model

#endif /* SWITCHEDMODEL_H_ */
