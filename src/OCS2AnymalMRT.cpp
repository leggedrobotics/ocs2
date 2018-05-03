/*
 * OCS2AnymalMRT.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_interface/OCS2AnymalMRT.h"

namespace anymal {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
OCS2AnymalMRT::OCS2AnymalMRT(const std::string& pathToConfigFolder)

: BASE(ocs2_anymal_interface_t::Ptr( new ocs2_anymal_interface_t(pathToConfigFolder) ), "anymal")
{}


} // end of namespace anymal


