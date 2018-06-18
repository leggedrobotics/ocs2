/*
 * OCS2AnymalMRT.h
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#ifndef OCS2ANYMALMRT_H_
#define OCS2ANYMALMRT_H_

#include <ocs2_quadruped_interface/OCS2QuadrupedMRT.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

namespace anymal {

class OCS2AnymalMRT : public switched_model::OCS2QuadrupedMRT<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2AnymalMRT> Ptr;

	typedef switched_model::OCS2QuadrupedMRT<12> BASE;

	typedef OCS2AnymalInterface ocs2_anymal_interface_t;


	OCS2AnymalMRT(const std::string& pathToConfigFolder);

	~OCS2AnymalMRT() = default;

};

} // end of namespace anymal

#endif /* OCS2ANYMALMRT_H_ */
