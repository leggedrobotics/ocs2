/*
 * MRT_ROS_Anymal.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_interface/MRT_ROS_Anymal.h"

namespace anymal {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MRT_ROS_Anymal::MRT_ROS_Anymal(const std::string& pathToConfigFolder)

: BASE(ocs2_anymal_interface_t::Ptr( new ocs2_anymal_interface_t(pathToConfigFolder) ), "anymal")
{}


} // end of namespace anymal
