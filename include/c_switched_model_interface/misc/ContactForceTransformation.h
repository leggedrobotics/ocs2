/*
 * ContactForceTransformation.h
 *
 *  Created on: Feb 23, 2016
 *      Author: farbod
 */

#ifndef HYQ_CONTACTFORCETRANSFORMATION_H_
#define HYQ_CONTACTFORCETRANSFORMATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <array>

#include "kinematics/SwitchedModelKinematics.h"

namespace switched_model {

class ContactForceTransformation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef Eigen::Matrix<double,18,1>     generalized_coordinate_t;
	typedef Eigen::Matrix<double,12,1>     contact_vector_t;
	typedef std::array<Eigen::Vector3d, 4> contact_array_t;

	ContactForceTransformation() {}
	~ContactForceTransformation() {}


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	/*
	 * transform the contact forces presented in the Origin frame to the nonorthogonal representation which
	 * its X and Y coordinates are parallel to the Base frame's X and Y and the its Z coordinate is in the
	 * direction of footToCoM in the World Frame.
	 */

	/******************************************************************************************************/
	void toNonorthogonalForces(const generalized_coordinate_t& generalizedCoordinate, const contact_vector_t& o_contactForces,
			contact_vector_t& nonorthogonalContactForces)  {

		// calculate the nonorthogonal coordinate base in the World frame
		std::array<Eigen::Matrix3d,4> nonorthogonalCoordinateBaseArray = nonorthogonalCoordinateBases(generalizedCoordinate);

		// Contact forces in non-orthogonal world coordinate
		Eigen::Vector3d w_conactForce;
		for(size_t i=0; i<4; i++) {
			// contact force in the World frame
			w_conactForce = b_R_o_*o_contactForces.segment<3>(3*i);
			// contact force in nonorthogonal coordinate
			nonorthogonalContactForces.segment<3>(3*i) = nonorthogonalCoordinateBaseArray[i].inverse()*w_conactForce;
		}
	}

	/******************************************************************************************************/
	void toNonorthogonalForces(const generalized_coordinate_t& generalizedCoordinate, const contact_array_t& o_contactForces,
			contact_array_t& nonorthogonalContactForces)  {

		// calculate the nonorthogonal coordinate base in the World frame
		std::array<Eigen::Matrix3d, 4> nonorthogonalCoordinateBaseArray = nonorthogonalCoordinateBases(generalizedCoordinate);

		// Contact forces in non-orthogonal world coordinate
		Eigen::Vector3d w_conactForce;
		for(size_t i=0; i<4; i++) {
			// contact force in the World frame
			w_conactForce = b_R_o_*o_contactForces[i];
			// contact force in nonorthogonal coordinate
			nonorthogonalContactForces[i] = nonorthogonalCoordinateBaseArray[i].inverse()*w_conactForce;
		}
	}


	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	/*
	 * transform the contact forces presented in the nonorthogonal representation (which
	 * its X and Y coordinates are parallel to the Base frame's X and Y and the its Z coordinate is in the
	 * direction of footToCoM in the World Frame) to the Origin frame coordinates.
	 */

	/******************************************************************************************************/
	void fromNonorthogonalForces(const generalized_coordinate_t& generalizedCoordinate, const contact_vector_t& nonorthogonalContactForces,
			contact_vector_t& o_contactsVector)  {

		// calculate the nonorthogonal coordinate base in the World frame
		std::array<Eigen::Matrix3d, 4> nonorthogonalCoordinateBaseArray = nonorthogonalCoordinateBases(generalizedCoordinate);

		// Contact forces in the Origin coordinate
		Eigen::Vector3d w_conactForce;
		for(size_t i=0; i<4; i++) {
			// contact force in the World frame
			w_conactForce = nonorthogonalCoordinateBaseArray[i]*nonorthogonalContactForces.segment<3>(3*i);
			// contact force in the Origin frame
			o_contactsVector.segment<3>(3*i) = b_R_o_.transpose()*w_conactForce;
		}
	}

	/******************************************************************************************************/
	void fromNonorthogonalForces(const generalized_coordinate_t& generalizedCoordinate, const contact_array_t& nonorthogonalContactForces,
			contact_array_t& o_contactsVector)  {

		// calculate the nonorthogonal coordinate base in the World frame
		std::array<Eigen::Matrix3d, 4> nonorthogonalCoordinateBaseArray = nonorthogonalCoordinateBases(generalizedCoordinate);

		// Contact forces in the Origin coordinate
		Eigen::Vector3d w_conactForce;
		for(size_t i=0; i<4; i++) {
			// contact force in the World frame
			w_conactForce = nonorthogonalCoordinateBaseArray[i]*nonorthogonalContactForces[i];
			// contact force in the Origin frame
			o_contactsVector[i] = b_R_o_.transpose()*w_conactForce;
		}
	}


private:

	/******************************************************************************************************/
	/******************************************************************************************************/
	/******************************************************************************************************/
	/*
	 * calculate the nonorthogonal coordinate base in the World frame
	 */
	std::array<Eigen::Matrix3d,4> nonorthogonalCoordinateBases(const generalized_coordinate_t& generalizedCoordinate)  {

		std::array<Eigen::Matrix3d, 4> nonorthogonalCoordinateBaseArray;

		// update the kinematics
		forwardKinematics_.update(generalizedCoordinate);

		// Origin to base rotation matrix
		b_R_o_ = forwardKinematics_.rotationMatrixOrigintoBase();

		// CoM position in the Base frame
		Eigen::Vector3d b_comPosition;
		forwardKinematics_.ComPositionBaseFrame(generalizedCoordinate.tail<12>(), b_comPosition);
		// CoM position in the Origin frame
		Eigen::Vector3d o_comPosition = b_R_o_.transpose()*b_comPosition + generalizedCoordinate.segment<3>(3);


		Eigen::Vector3d o_footPosition;

		// calculate the nonorthogonal coordinate base in the World frame
		for(size_t i=0; i<4; i++) {
			// foot position in the Origin frame
			forwardKinematics_.footPositionOriginFrame(i, o_footPosition);
			// foot to CoM vector in the World frame
			Eigen::Vector3d w_footToCoM  = b_R_o_*(o_comPosition - o_footPosition);
			nonorthogonalCoordinateBaseArray[i] << Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,1,0), w_footToCoM.normalized();
		}

		return nonorthogonalCoordinateBaseArray;
	}


	SwitchedModelKinematics forwardKinematics_;
	Eigen::Matrix3d b_R_o_;
};

}  // end of switched_model namespace

#endif /* HYQ_CONTACTFORCETRANSFORMATION_H_ */
