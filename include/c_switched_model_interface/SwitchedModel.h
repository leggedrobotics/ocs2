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

template< size_t JOINT_COORD_SIZE >
class SwitchedModel
{
public:
	enum
	{
		BASE_COORDINATE_SIZE        = 6,
		JOINT_COORDINATE_SIZE       = JOINT_COORD_SIZE,
		GENERALIZED_COORDINATE_SIZE = BASE_COORDINATE_SIZE + JOINT_COORD_SIZE
	};

	typedef Eigen::Matrix<double,GENERALIZED_COORDINATE_SIZE,1> generalized_coordinate_t;
	typedef Eigen::Matrix<double,JOINT_COORDINATE_SIZE,1>       joint_coordinate_t;
	typedef Eigen::Matrix<double,BASE_COORDINATE_SIZE,1>        base_coordinate_t;
	typedef float scalar_low_t;
	typedef double scalar_high_t;
};

#endif /* SWITCHEDMODEL_H_ */
