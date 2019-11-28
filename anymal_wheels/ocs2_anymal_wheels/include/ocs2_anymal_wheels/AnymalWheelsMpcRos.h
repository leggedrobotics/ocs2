/*
 * AnymalWheelsMpcRos.h
 *
 *  Created on: Nov 27, 2019
 *      Author: Marko Bjelonic
 */

#ifndef MPC_ROS_ANYMAL_H_
#define MPC_ROS_ANYMAL_H_

#include <ocs2_quadruped_interface/MPC_ROS_Quadruped.h>

namespace anymal {

using AnymalWheelsMpcRos = switched_model::MPC_ROS_Quadruped<16>;

} // end of namespace anymal

#endif /* MPC_ROS_ANYMAL_H_ */
