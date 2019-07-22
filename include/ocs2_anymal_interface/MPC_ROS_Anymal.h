/*
 * MPC_ROS_Anymal.h
 *
 *  Created on: Jun 18, 2018
 *      Author: farbod
 */

#ifndef MPC_ROS_ANYMAL_H_
#define MPC_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MPC_ROS_Quadruped.h>

#include "ocs2_anymal_interface/OCS2AnymalInterface.h"

namespace anymal {

class MPC_ROS_Anymal : public switched_model::MPC_ROS_Quadruped<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<MPC_ROS_Anymal> Ptr;

	typedef switched_model::MPC_ROS_Quadruped<12> BASE;

	typedef OCS2AnymalInterface ocs2_anymal_interface_t;

	MPC_ROS_Anymal(const std::string& pathToConfigFolder) : BASE(ocs2_anymal_interface_t::Ptr( new ocs2_anymal_interface_t(pathToConfigFolder) ), "anymal")
	{}

	~MPC_ROS_Anymal() = default;

};

} // end of namespace anymal

#endif /* MPC_ROS_ANYMAL_H_ */
