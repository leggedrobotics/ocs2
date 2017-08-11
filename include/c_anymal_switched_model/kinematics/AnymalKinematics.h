/*
 * AnymalKinematics.h
 *
 *  Created on: Aug 11, 2017
 *      Author: farbod
 */

#ifndef ANYMAL_ANYMALKINEMATICS_H_
#define ANYMAL_ANYMALKINEMATICS_H_

#include <c_switched_model_interface/ModelKinematicsBase.h>

#include <iit/robots/anymal/transforms.h>
#include <iit/robots/anymal/jacobians.h>

namespace anymal
{

class AnymalKinematics : public ModelKinematicsBase<AnymalKinematics, 12>
{
public:
	enum { LF=0,  RF=1,  LH=2,  RH=3,
		JOINT_COORDINATE_SIZE=12
	};

	AnymalKinematics() {}
	~AnymalKinematics() {}

	void FootPositionBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Vector3d& footPosition);

	void FootJacobainBaseFrame(const joint_coordinate_t& jointCoordinate,
			const size_t& footIndex, Eigen::Matrix<double,6,JOINT_COORDINATE_SIZE>& footJacobain);
private:

};

}  // end of anymal namespace

#endif /* ANYMAL_ANYMALKINEMATICS_H_ */
