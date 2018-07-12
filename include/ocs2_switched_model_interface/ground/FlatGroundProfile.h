/*
 * FlatGroundProfile.h
 *
 *  Created on: Mar 19, 2018
 *      Author: farbod
 */


#ifndef FLATGROUNDPROFILE_H_
#define FLATGROUNDPROFILE_H_

#include "ocs2_switched_model_interface/ground/GroundProfileBase.h"

namespace switched_model {

template <typename scalar_t = double>
class FlatGroundProfile : public GroundProfileBase<scalar_t>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<FlatGroundProfile> Ptr;

	typedef GroundProfileBase<scalar_t> BASE;
	typedef typename BASE::vector_3d_t vector_3d_t;

	FlatGroundProfile()
	: FlatGroundProfile(0.0)
	{}

	FlatGroundProfile(const scalar_t& groundHight)
	: groundHight_(groundHight)
	{}

	~FlatGroundProfile() = default;

	scalar_t getGroundHight(const vector_3d_t& p) override {
		return groundHight_;
	}

	void setGroundHight(const scalar_t& groundHight) {
		groundHight_ = groundHight;
	}

protected:
	scalar_t groundHight_;
};

} //end of namespace switched_model

#endif /* FLATGROUNDPROFILE_H_ */
