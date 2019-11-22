/*
 * AnymalBearMpcRos.h
 *
 *  Created on: Jun 18, 2018
 *      Author: farbod
 */

#ifndef MPC_ROS_ANYMAL_H_
#define MPC_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MPC_ROS_Quadruped.h>

namespace anymal {

using AnymalBearMpcRos = switched_model::MPC_ROS_Quadruped<12>;

} // end of namespace anymal

#endif /* MPC_ROS_ANYMAL_H_ */
