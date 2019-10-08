/*
 * GroundProfileBase.h
 *
 *  Created on: Mar 19, 2018
 *      Author: farbod
 */

#ifndef GROUNDPROFILEBASE_H_
#define GROUNDPROFILEBASE_H_

#include <memory>
#include <Eigen/Dense>

namespace switched_model {

template <typename scalar_t = double>
class GroundProfileBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<GroundProfileBase> Ptr;

	typedef Eigen::Matrix<scalar_t,3,1> vector_3d_t;

	GroundProfileBase() = default;

	virtual ~GroundProfileBase() = default;

	virtual scalar_t getGroundHight(const vector_3d_t& p) = 0;

protected:

};

} //end of namespace switched_model

#endif /* GROUNDPROFILEBASE_H_ */
