/*
 * OCS2AnymalMPC.h
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#ifndef OCS2ANYMALMPC_H_
#define OCS2ANYMALMPC_H_

#include <ocs2_quadruped_interface/OCS2QuadrupedMPC.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

namespace anymal {

class OCS2AnymalMPC : public switched_model::OCS2QuadrupedMPC<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<OCS2AnymalMPC> Ptr;

	typedef switched_model::OCS2QuadrupedMPC<12> BASE;

	typedef OCS2AnymalInterface ocs2_anymal_interface_t;

	OCS2AnymalMPC(const std::string& pathToConfigFolder);

	~OCS2AnymalMPC() = default;

};

} // end of namespace anymal

#endif /* OCS2ANYMALMPC_H_ */
