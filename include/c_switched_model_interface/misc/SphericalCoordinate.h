/*
 * SphericalCoordinate.h
 *
 *  Created on: Jun 29, 2016
 *      Author: farbod
 */

#ifndef HYQ_SPHERICALCOORDINATE_H_
#define HYQ_SPHERICALCOORDINATE_H_

#include <Eigen/Dense>

namespace hyq {

class SphericalCoordinate
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SphericalCoordinate() {}
	~SphericalCoordinate() {}

	// the transformation for a vector expressed in the Spherical coordinate (r,theta,phi) to the Cartesian coordinate (x,y,z)
	static Eigen::Matrix3d FromSphericalToCartesian(const Eigen::Vector3d& cartesianPoint) {

		double r   = cartesianPoint.norm();
		double pho = cartesianPoint.head<2>().norm();

		double sinTheta = pho/r;
		double cosTheta = cartesianPoint(2)/r;

		double sinPhi = 0.0;
		double cosPhi = 1.0;
		if (sinTheta > Eigen::NumTraits<double>::epsilon()) {
			sinPhi = cartesianPoint(1)/pho;
			cosPhi = cartesianPoint(0)/pho;
		}


		Eigen::Matrix3d transformation;
		transformation <<
				sinTheta*cosPhi, cosTheta*cosPhi, -sinPhi,
				sinTheta*sinPhi, cosTheta*sinPhi, cosPhi,
				cosTheta, -sinTheta, 0.0;

		return transformation;
	}


	// the transformation for a vector expressed in the Cartesian coordinate (x,y,z) to the Spherical coordinate (r,theta,phi)
	static Eigen::Matrix3d FromCartesianToSpherical(const Eigen::Vector3d& cartesianPoint) {

		double r   = cartesianPoint.norm();
		double pho = cartesianPoint.head<2>().norm();

		double sinTheta = pho/r;
		double cosTheta = cartesianPoint(2)/r;

		double sinPhi = 0.0;
		double cosPhi = 1.0;
		if (sinTheta > Eigen::NumTraits<double>::epsilon()) {
			sinPhi = cartesianPoint(1)/pho;
			cosPhi = cartesianPoint(0)/pho;
		}

		Eigen::Matrix3d transformation;
		transformation <<
				sinTheta*cosPhi, sinTheta*sinPhi, cosTheta,
				cosTheta*cosPhi, cosTheta*sinPhi, -sinTheta,
				-sinPhi, cosPhi, 0.0;

		return transformation;
	}


	static Eigen::Matrix3d SphericalToCartesianTransformationDerivative(const Eigen::Vector3d& cartesianPoint, size_t coordinateIndex) {

		Eigen::Matrix3d transformationDerivative;

		const double& x = cartesianPoint[0];
		const double& y = cartesianPoint[1];
		const double& z = cartesianPoint[2];
		double r   = cartesianPoint.norm();
		double pho = cartesianPoint.head<2>().norm();

		switch (coordinateIndex)  {
		case 0:
			transformationDerivative <<
				1/r*(1-pow(x/r,2)),  z/pho/r*(1-pow(x/r,2)-pow(x/pho,2)),     x*y/pow(pho,3),
				-x*y/pow(r,3),		 -x*y*z/pho/r*(1/pow(pho,2)+1/pow(r,2)),  pow(y,2)/pow(pho,3),
				-z*x/pow(r,3),       -x*pow(z,2)/pho/pow(r,3),				  0.0;
			break;

		case 1:
			transformationDerivative <<
				-x*y/pow(r,3),       -x*y*z/pho/r*(1/pow(pho,2)+1/pow(r,2)),  -pow(x,2)/pow(pho,3),
				1/r*(1-pow(y/r,2)),  z/pho/r*(1-pow(y/pho,2)-pow(y/r,2)),     -x*y/pow(pho,3),
				-z*y/pow(r,3),       -y*pow(z,2)/pho/pow(r,3),                0.0;
			break;

		case 2:
			transformationDerivative <<
				-x*z/pow(r,3),        x*pho/pow(r,3),  0.0,
				-y*z/pow(r,3),        y*pho/pow(r,3),  0.0,
				pow(pho,2)/pow(r,3),  pho*z/pow(r,3),  0.0;
			break;

		}

		return transformationDerivative;
	}


private:


};

}  // end of hyq namespace

#endif /* HYQ_SPHERICALCOORDINATE_H_ */
