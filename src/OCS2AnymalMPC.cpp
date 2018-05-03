/*
 * OCS2AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_interface/OCS2AnymalMPC.h"

namespace anymal {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OCS2AnymalMPC::OCS2AnymalMPC(const std::string& pathToConfigFolder)

: BASE(ocs2_anymal_interface_t::Ptr( new ocs2_anymal_interface_t(pathToConfigFolder) ), "anymal")
{}


} // end of namespace anymal



