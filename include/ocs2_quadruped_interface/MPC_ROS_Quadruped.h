/*
 * MPC_ROS_Quadruped.h
 *
 *  Created on: May 27, 2018
 *      Author: farbod
 */

#ifndef MPC_ROS_QUADRUPED_H_
#define MPC_ROS_QUADRUPED_H_

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

namespace switched_model {

template<size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
using MPC_ROS_Quadruped = ocs2::MPC_ROS_Interface<STATE_DIM, INPUT_DIM>;

} // namespace switched_model

#endif /* MPC_ROS_QUADRUPED_H_ */
